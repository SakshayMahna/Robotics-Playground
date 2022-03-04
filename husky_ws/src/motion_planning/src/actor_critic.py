import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.initializers import RandomUniform


class Critic:
    def __init__(self, obs_dim, action_dim, learning_rate=0.001):
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.model = self.make_network()
        self.optimizer = keras.optimizers.Adam(learning_rate)
        # self.model.compile(loss="mse", optimizer=self.optimizer)

    def make_network(self):
        obs_input = keras.Input(shape=self.obs_dim, dtype="float32", name="obs")
        action_input = keras.Input(
            shape=self.action_dim, dtype="float32", name="action"
        )

        # layer 0 - with obs input
        w_range = 1 / np.sqrt(self.obs_dim)
        lr_0 = keras.layers.Dense(
            400,
            activation="relu",
            name="c_lr_0",
            kernel_initializer=RandomUniform(-w_range, w_range),
        )(obs_input)

        # layer 1 with concatenated input of [lr_0, action]
        lr_1_input = keras.layers.concatenate([lr_0, action_input])
        w_range = 1 / np.sqrt(400.0)
        lr_1 = keras.layers.Dense(
            300,
            activation="relu",
            name="c_lr_1",
            kernel_initializer=RandomUniform(-w_range, w_range),
        )(lr_1_input)

        # final layers with linear activation
        w_range = 0.003
        q_val = keras.layers.Dense(
            1,
            activation="linear",
            name="q_val",
            kernel_initializer=RandomUniform(-w_range, w_range),
        )(lr_1)

        model = keras.Model(inputs=[obs_input, action_input], outputs=q_val)
        return model

    @tf.function
    def estimate_q(self, obs, action):
        obs = tf.reshape(obs, (-1, self.obs_dim))
        action = tf.reshape(action, (-1, self.action_dim))
        return self.model([obs, action])


class Actor:
    def __init__(self, obs_dim, action_dim, action_gain, learning_rate=0.0001):
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.action_gain = action_gain
        self.model = self.make_network()
        self.optimizer = keras.optimizers.Adam(learning_rate)

    def make_network(self):
        obs_input = keras.Input(shape=self.obs_dim, dtype="float32", name="obs")

        # layer 0 - with obs input
        w_range = 1 / np.sqrt(self.obs_dim)
        lr_0 = keras.layers.Dense(
            400,
            activation="relu",
            name="a_lr_0",
            kernel_initializer=RandomUniform(-w_range, w_range),
        )(obs_input)

        # layer 1
        w_range = 1 / np.sqrt(400.0)
        lr_1 = keras.layers.Dense(
            300,
            activation="relu",
            name="a_lr_1",
            kernel_initializer=RandomUniform(-w_range, w_range),
        )(lr_0)

        # action layer
        w_range = 0.003
        action = self.action_gain * keras.layers.Dense(
            self.action_dim,
            activation="tanh",
            name="action",
            kernel_initializer=RandomUniform(-w_range, w_range),
        )(lr_1)

        model = keras.Model(inputs=obs_input, outputs=action)
        return model

    @tf.function
    def act(self, obs):
        obs = tf.reshape(obs, (-1, self.obs_dim))
        return self.model(obs)