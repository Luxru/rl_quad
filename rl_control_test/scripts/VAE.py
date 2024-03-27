import torch
import torch.nn as nn
import torch.nn.functional as F

"""输入的数据是2维的特征向量
传统的AE编码器输出的code是一个数值张量，而在VAE输出的code是随机变量组成的张量
"""

class VAE(nn.Module):
    def __init__(self, input_dim, hidden_dim, latent_dim):
        super(VAE, self).__init__()

        self.encoder = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU()
        )

        self.mu = nn.Linear(hidden_dim, latent_dim)
        self.log_var = nn.Linear(hidden_dim, latent_dim)

        self.decoder = nn.Sequential(
            nn.Linear(latent_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, input_dim),
            nn.Sigmoid()
        )

    def encode(self, x):
        h = self.encoder(x)
        mu = self.mu(h)
        log_var = self.log_var(h)
        return mu, log_var

    def reparameterize(self, mu, log_var):
        std = torch.exp(0.5 * log_var)
        eps = torch.randn_like(std)
        z = mu + eps * std
        return z

    def decode(self, z):
        x_recon = self.decoder(z)
        return x_recon

    def forward(self, x):
        mu, log_var = self.encode(x)
        z = self.reparameterize(mu, log_var)
        x_recon = self.decode(z)
        return x_recon, mu, log_var


    # 定义损失函数
    def vae_loss(self, x, x_recon, mu, log_var):
        recon_loss = F.binary_cross_entropy(x_recon, x, reduction='sum')
        kl_div = -0.5 * torch.sum(1 + log_var - mu.pow(2) - log_var.exp())
        return recon_loss + kl_div


#
input_dim = 2
hidden_dim = 128
latent_dim = 64
batch_size = 32
num_epochs = 10
learning_rate = 1e-3

model = VAE(input_dim, hidden_dim, latent_dim)
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

# 假设我们有一个2维特征向量的数据集，这个自己可以调整，根据需要调整输入输出的维度
data = torch.randn(1000, input_dim)

# 训练循环
for epoch in range(num_epochs):
    for i in range(0, len(data), batch_size):
        batch = data[i:i + batch_size]
        x_recon, mu, log_var = model(batch)
        loss = model.vae_loss(batch, x_recon, mu, log_var)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    print(f'Epoch [{epoch + 1}/{num_epochs}], Loss: {loss.item():.4f}')