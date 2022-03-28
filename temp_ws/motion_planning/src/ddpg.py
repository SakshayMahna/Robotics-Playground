import os
import numpy as np
import tensorflow as tf
from tensorflow import keras

from actor_critic import Actor, Critic
from replay_buffer import ReplayBuffer

REPLAY_BUFFER_SIZE = 100000
REPLAY_START_SIZE = 10000
BATCH_SIZE = 128
GAMMA = 0.99

class DDPG:
    def __init__(
        self,
        env,
        obs_dim,
        action_dim,
        action_gain=1,
        actor_learning_rate=0.0001,
        critic_learning_rate=0.001,
        gamma=0.99,
        tau=0.001,
    ):
        self.environment = env
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.gamma = gamma
        self.tau = tau
        self.time_step = 0

        # make main networks
        self.actor = Actor(obs_dim, action_dim, action_gain, actor_learning_rate)
        self.critic = Critic(obs_dim, action_dim, critic_learning_rate)

        # make target networks
        self.target_actor = Actor(obs_dim, action_dim, action_gain)
        self.target_actor.model.set_weights(self.actor.model.get_weights())
        self.target_critic = Critic(obs_dim, action_dim)
        self.target_critic.model.set_weights(self.critic.model.get_weights())

        # set replay buffer
        self.replay_buffer = ReplayBuffer(REPLAY_BUFFER_SIZE)

    def action(self, obs):
        return self.actor.act(obs)[0]

    def train(self):
        batch = self.replay_buffer.sample_batch(BATCH_SIZE)
        self.update_networks(batch)

    def perceive(self, state, action, reward, next_state, done):
        # Store transition (s_t,a_t,r_t,s_{t+1}) in replay buffer
        self.replay_buffer.add([state, action, reward, done, next_state])
        if self.replay_buffer.size() == REPLAY_START_SIZE:
            print('\n---------------Start training---------------')
        # Store transitions to replay start size then start training
        if self.replay_buffer.size() > REPLAY_START_SIZE:
            self.time_step += 1
            self.train()

        if self.time_step % 10000 == 0 and self.time_step > 0:
            self.save_model("./results")

        return self.time_step

    @tf.function
    def update_networks(self, batch):
        """ runs all updates from provided training batch """
        s, a, r, t, next_s = (tf.cast(i, tf.float32) for i in batch)
        self.update_critic(s, a, r, next_s)
        self.update_actor(s, a, r, next_s)
        self.update_target(self.actor.model, self.target_actor.model)
        self.update_target(self.critic.model, self.target_critic.model)

    @tf.function
    def update_critic(self, s, a, r, next_s):
        """ minimize td-loss from target """
        # td estimate based on targets' behavior
        target_future_actions = self.target_actor.act(next_s)
        target_future_qs = self.target_critic.estimate_q(next_s, target_future_actions)
        target_current_qs = r + self.gamma * target_future_qs

        # update main critic
        main_current_qs = self.critic.model([s, a])
        loss = keras.losses.mse(target_current_qs, main_current_qs)

        model_vars = self.critic.model.trainable_variables

        dloss_dcrit = tf.gradients(loss, model_vars)
        self.critic.optimizer.apply_gradients(zip(dloss_dcrit, model_vars))

    @tf.function
    def update_actor(self, s, a, r, next_s):
        """ dq_dtheta = dq_da * da_dtheta"""
        # first, finding dq_da
        proposed_a = self.actor.model(s)
        q = self.critic.model([s, proposed_a])
        dq_da = tf.gradients(q, proposed_a)[0]

        # second, finding dq_da * da_dtheta
        model_vars = self.actor.model.trainable_variables
        dq_dtheta = tf.gradients(proposed_a, model_vars, grad_ys=-dq_da)

        # updating the model
        self.actor.optimizer.apply_gradients(zip(dq_dtheta, model_vars))

    @tf.function
    def update_target(self, main_model, target_model):
        """ target = tau*main + (1-tau)*target """
        for model_weight, target_weight in zip(
            main_model.weights, target_model.weights
        ):
            target_weight.assign(
                self.tau * model_weight + (1 - self.tau) * target_weight
            )

    def save_model(self, save_dir):
        """ saves the main and target networks"""
        self.actor.model.save_weights(os.path.join(save_dir, f"actor_{self.time_step}"))
        self.critic.model.save_weights(os.path.join(save_dir, f"critic_{self.time_step}"))
        self.target_actor.model.save_weights(os.path.join(save_dir, f"target_actor_{self.time_step}"))
        self.target_critic.model.save_weights(os.path.join(save_dir, f"target_critic_{self.time_step}"))