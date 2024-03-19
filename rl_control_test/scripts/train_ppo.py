from stable_baselines3 import PPO
import uav_gym
from stable_baselines3.common.env_checker import check_env


env = uav_gym.CustomEnv()
# It will check your custom environment and output additional warnings if needed
check_env(env)
model = PPO("MlpPolicy",
            env,verbose=1,
            device="cuda",
            n_steps=250,
            tensorboard_log="./tf_log",
            gamma=0.99,  # lower 0.9 ~ 0.99
            # n_steps=math.floor(cfg['env']['max_time'] / cfg['env']['ctl_dt']),
            ent_coef=0.00,
            learning_rate=3e-4,
            vf_coef=0.5,
            max_grad_norm=0.5,
            n_epochs=10,
            clip_range=0.2)
model.learn(total_timesteps=25000000)
model.save("./ppo") 