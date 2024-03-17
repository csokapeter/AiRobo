import os
from PPO import PPO
from datetime import date
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.environment import UnityEnvironment


def agent_info(env):
    behavior_name = list(env.behavior_specs.keys())[0]
    spec = env.behavior_specs[behavior_name]
    decision_steps, terminal_steps = env.get_steps(behavior_name)
    agent_id = list(decision_steps)[0]

    return behavior_name, spec, decision_steps, terminal_steps, agent_id


def train(env, model, opts):
    print('Training')
    behavior_name, spec, decision_steps, terminal_steps, agent_id = agent_info(env)
    rewards = []
    agent_reward = 0
    done = False

    f_out = open(f'training_log_L{opts["level"]}_{opts["today"]}.txt', 'a')
    folder = f'weights_L{opts["level"]}_{opts["today"]}'
    if not os.path.exists(folder):
        os.makedirs(folder)

    for episode in range(opts['init_episode'], opts['max_episodes']):
        done = False
        agent_reward = 0

        behavior_name, spec, decision_steps, terminal_steps, agent_id = agent_info(env)
        current_obs = decision_steps[agent_id].obs[0]
        while not done:
            action = ActionTuple(model.select_action(current_obs).reshape(1, opts['action_dim']))
            env.set_actions(behavior_name, action)
            env.step()

            decision_steps, terminal_steps = env.get_steps(behavior_name)
            if agent_id in terminal_steps:
                aftermath = terminal_steps[agent_id]
                done = True
            else:
                aftermath = decision_steps[agent_id]

            new_obs = aftermath.obs[0]
            reward = aftermath.reward
            model.store_obs(reward, done)
            agent_reward += reward
            current_obs = new_obs

        rewards.append(agent_reward)
        print(f'{episode = } reward: {agent_reward}')

        if episode % opts['update_freq'] == 0:
            print('updating goal weights')
            actor_loss_g, critic_loss_g, loss_g = model.update(current_obs)

        if episode % opts['reward_stat_freq'] == 0:
            model.save_weights(f'L{opts["level"]}_e{episode}_{opts["today"]}', folder)
            mean_reward = np.mean(rewards[-opts['reward_stat_freq']:])
            print(f'Mean reward between {episode - opts["reward_stat_freq"]}-{episode}: {mean_reward}\n')
            for reward in rewards:
                f_out.write(f'{reward}\n')
            rewards.clear()

        if episode % opts['action_std_decay_freq'] == 0:
            opts['action_std'] -= opts['action_std_decay_rate']
            if opts['action_std'] < opts['min_action_std']:
                opts['action_std'] = opts['min_action_std']
            model.set_action_std(opts['action_std'])

    f_out.close()
    env.close()


def inference(env, model, opts):
    print('Inference')
    behavior_name, spec, decision_steps, terminal_steps, agent_id = agent_info(env)
    done = False

    while True:
        done = False
        reward = 0

        decision_steps, terminal_steps = env.get_steps(behavior_name)
        current_obs = decision_steps[agent_id].obs[0]

        while not done:
            action = ActionTuple(model.select_action(current_obs).reshape(1, opts['action_dim']))
            
            env.set_actions(behavior_name, action)
            env.step()

            decision_steps, terminal_steps = env.get_steps(behavior_name)
            if agent_id in terminal_steps:
                aftermath = terminal_steps[agent_id]
                done = True
            else:
                aftermath = decision_steps[agent_id]
            
            reward += aftermath.reward
            current_obs = aftermath.obs[0]
        
        print(f'{reward = }')


def main():
    print('Loading Unity environment...')
    env = UnityEnvironment()
    env.reset()

    use_training_mode = True

    opts = {
        'state_dim' : 3,
        'action_dim' : 2,
        'max_episodes' : 10_001,
        'reward_stat_freq' : 100,
        'init_episode' : 1,
        'update_freq' : 4,               # weights update frequency in number of episodes
        'lr' : 0.0003,
        'action_std' : 1,                # starting std for action distribution (Normal)
        'action_std_decay_rate' : 0.1,   # linearly decay action_std (action_std = action_std - action_std_decay_rate)
        'min_action_std' : 0.5,          # minimum action_std (stop decay after action_std <= min_action_std)
        'action_std_decay_freq' : 1500,  # action_std decay frequency (in num episodes)

        'today' : date.today().strftime('%Y-%m-%d'),
        'level' : '1'
    }

    ppo_agent = PPO(opts['state_dim'], opts['action_dim'], opts['lr'], opts['action_std'], hidden_size=128)
    # ppo_agent.load('L1_e7000_2024-03-14.pth', 'weights_L1_2024-03-14')

    if use_training_mode:
        train(env, ppo_agent, opts)
    else:
        inference(env, ppo_agent, opts)


if __name__ == '__main__':
    main()
