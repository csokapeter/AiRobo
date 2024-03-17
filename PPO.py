import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal


use_cuda = torch.cuda.is_available()
device   = 'cpu' #torch.device("cuda" if use_cuda else "cpu")
torch.set_num_threads(1)
torch.set_num_interop_threads(1)
print(f'using device: {device}')


def init_weights(m):
    if isinstance(m, nn.Linear):
        nn.init.normal_(m.weight, mean=0., std=0.1)
        nn.init.constant_(m.bias, 0.1)


class Memory:
    def __init__(self):
        self.log_probs = []
        self.values = []
        self.states = []
        self.actions = []
        self.rewards = []
        self.masks = []

    def clear(self):
        self.log_probs.clear()
        self.values.clear()
        self.states.clear()
        self.actions.clear()
        self.rewards.clear()
        self.masks.clear()


class ActorCritic(nn.Module):
    def __init__(self, num_inputs, num_outputs, hidden_size, action_std):
        super(ActorCritic, self).__init__()

        self.critic = nn.Sequential(
            nn.Linear(num_inputs, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, 1)
        )

        self.actor = nn.Sequential(
            nn.Linear(num_inputs, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, num_outputs),
            nn.Tanh()
        )

        self.action_std = action_std
        self.apply(init_weights)

    def forward(self, x):
        value = self.critic(x)
        mu = self.actor(x)
        std = torch.tensor([self.action_std], device=device).expand_as(mu)
        dist = Normal(mu, std)
        return dist, value


class PPO:
    def __init__(self, num_inputs, num_outputs, lr, action_std, hidden_size=128, ppo_epochs=8, mini_batch_size=128):
        self.num_inputs = num_inputs
        self.ppo_epochs = ppo_epochs
        self.mini_batch_size = mini_batch_size
        self.hidden_size = hidden_size

        self.model = ActorCritic(num_inputs, num_outputs, hidden_size, action_std).to(device)
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=lr)
        self.memory = Memory()


    def set_action_std(self, new_action_std):
        self.model.action_std = new_action_std ** 2
        print(f'new action std: {self.model.action_std}')


    def compute_gae(self, next_value, rewards, masks, values, gamma=0.99, tau=0.95):
        values = values + [next_value]
        gae = 0
        returns = []
        for step in reversed(range(len(rewards))):
            delta = rewards[step] + gamma * values[step + 1] * masks[step] - values[step]
            gae = delta + gamma * tau * masks[step] * gae
            returns.insert(0, gae + values[step])
        return returns


    def ppo_iter(self, mini_batch_size, states, actions, log_probs, returns, advantage):
        batch_size = states.size(0)
        for _ in range(max(batch_size // mini_batch_size, 1)):
            rand_ids = np.random.randint(0, batch_size, mini_batch_size)
            yield states[rand_ids, :], actions[rand_ids, :], log_probs[rand_ids, :], returns[rand_ids, :], advantage[rand_ids, :]


    def ppo_update(self, ppo_epochs, mini_batch_size, states, actions, log_probs, returns, advantages, clip_param=0.2):
        actor_loss, critic_loss, loss = 0, 0, 0
        for _ in range(ppo_epochs):
            for state, action, old_log_probs, return_, advantage in self.ppo_iter(mini_batch_size, states, actions, log_probs, returns, advantages):
                dist, value = self.model(state)
                #entropy = dist.entropy().mean()
                new_log_probs = dist.log_prob(action)

                ratio = (new_log_probs - old_log_probs).exp()
                surr1 = ratio * advantage
                surr2 = torch.clamp(ratio, 1.0 - clip_param, 1.0 + clip_param) * advantage

                actor_loss  = - torch.min(surr1, surr2).mean()
                critic_loss = (return_ - value).pow(2).mean()

                #uses gradient descent algorithm - we want to maximize the actor 'loss' (hence the *-1), minimize the critic loss
                loss = 0.5 * critic_loss + actor_loss #- 0.001 * entropy

                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()

        self.memory.clear()
        return actor_loss, critic_loss, loss


    def select_action(self, state):
        self.state = torch.tensor(state, device=device).unsqueeze(0)
        self.dist, self.value = self.model(self.state)
        self.action = self.dist.sample()
        return self.action.squeeze().cpu().numpy()


    def store_obs(self, reward, done):
        log_prob = self.dist.log_prob(self.action)

        self.memory.log_probs.append(log_prob)
        self.memory.values.append(self.value)
        self.memory.rewards.append(torch.tensor([reward], device=device).unsqueeze(1))
        self.memory.masks.append(torch.tensor([1 - done], device=device).unsqueeze(1))
        self.memory.states.append(self.state)
        self.memory.actions.append(self.action)


    def update(self, next_state):
        next_state = torch.tensor(next_state, device=device)
        _, next_value = self.model(next_state)
        returns = self.compute_gae(next_value, self.memory.rewards, self.memory.masks, self.memory.values)

        returns = torch.cat(returns).detach()
        log_probs = torch.cat(self.memory.log_probs).detach()
        values = torch.cat(self.memory.values).detach()
        states = torch.cat(self.memory.states).detach()
        actions = torch.cat(self.memory.actions).detach()
        advantage = returns - values

        actor_loss, critic_loss, loss = self.ppo_update(self.ppo_epochs, self.mini_batch_size, states, actions, log_probs, returns, advantage)
        return actor_loss, critic_loss, loss


    def save_weights(self, filename, directory):
        torch.save(self.model.state_dict(), '%s/%s.pth' % (directory, filename))


    def load(self, filename, directory):
        state_dict = torch.load('%s/%s' % (directory, filename), map_location=lambda storage, loc: storage)
        additional_dim = self.num_inputs - len(state_dict['actor.0.weight'][0])
        print(f'loading {filename}')
        print(f'additional inputs compared to loaded weights: {additional_dim}')

        random_weights = torch.tensor(np.random.uniform(-0.5, 0.5,[self.hidden_size, additional_dim]), device=device)
        state_dict['actor.0.weight'] = torch.cat((state_dict['actor.0.weight'].to(device), random_weights), 1)
        state_dict['critic.0.weight'] = torch.cat((state_dict['critic.0.weight'][:,:self.num_inputs-additional_dim].to(device), random_weights, state_dict['critic.0.weight'][:,self.num_inputs-additional_dim:].to(device)), 1)

        self.model.load_state_dict(state_dict)
