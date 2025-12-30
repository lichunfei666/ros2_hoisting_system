# Git 代理配置指南

## 1. HTTP/HTTPS 代理配置

如果您的Git仓库使用的是HTTPS URL（如：https://github.com/username/repo.git），可以通过以下方式配置代理：

### 1.1 全局代理配置

```bash
# 设置HTTP代理（将proxy-server和port替换为您的VPN代理服务器地址和端口）
git config --global http.proxy http://proxy-server:port

# 设置HTTPS代理（将proxy-server和port替换为您的VPN代理服务器地址和端口）
git config --global https.proxy https://proxy-server:port
```

### 1.2 针对特定主机的代理配置

如果您只想为GitHub设置代理，可以使用以下命令：

```bash
# 设置GitHub的HTTP代理
git config --global http.https://github.com.proxy http://proxy-server:port

# 设置GitHub的HTTPS代理
git config --global https.https://github.com.proxy https://proxy-server:port
```

### 1.3 取消代理配置

```bash
# 取消全局HTTP代理
git config --global --unset http.proxy

# 取消全局HTTPS代理
git config --global --unset https.proxy

# 取消GitHub的HTTP代理
git config --global --unset http.https://github.com.proxy

# 取消GitHub的HTTPS代理
git config --global --unset https.https://github.com.proxy
```

## 2. SSH 代理配置

如果您的Git仓库使用的是SSH URL（如：git@github.com:username/repo.git），需要配置SSH代理。

### 2.1 修改SSH配置文件

```bash
# 编辑SSH配置文件
vim ~/.ssh/config
```

在配置文件中添加以下内容（将proxy-server和port替换为您的VPN代理服务器地址和端口）：

```
Host github.com
  HostName github.com
  User git
  # 如果使用SOCKS5代理
  # ProxyCommand nc -x proxy-server:port %h %p
  # 如果使用HTTP代理
  # ProxyCommand corkscrew proxy-server port %h %p
  # 如果使用HTTPS代理
  # ProxyCommand corkscrew proxy-server port %h %p
```

### 2.2 安装必要工具

如果您使用corkscrew（用于HTTP/HTTPS代理），需要先安装：

```bash
sudo apt-get update
sudo apt-get install corkscrew
```

如果您使用nc（用于SOCKS5代理），通常已经安装在Linux系统中。

## 3. 检查代理配置

### 3.1 检查Git配置

```bash
git config --list
```

### 3.2 测试代理连接

```bash
# 测试HTTP/HTTPS代理
curl -x http://proxy-server:port https://github.com

# 测试SSH代理
ssh -T git@github.com
```

## 4. 注意事项

1. 确保您的VPN代理服务器地址和端口正确
2. 根据您的VPN类型（HTTP/HTTPS/SOCKS5）选择合适的代理配置方式
3. 如果您的VPN需要身份验证，可以使用以下格式：
   ```bash
   # HTTP代理（带身份验证）
   git config --global http.proxy http://username:password@proxy-server:port
   
   # HTTPS代理（带身份验证）
   git config --global https.proxy https://username:password@proxy-server:port
   ```
4. 对于SSH配置文件中的身份验证，可以在ProxyCommand中添加用户名和密码

## 5. 示例配置

### 5.1 使用HTTP代理的示例

```bash
# 全局HTTP代理配置
git config --global http.proxy http://127.0.0.1:1080
git config --global https.proxy http://127.0.0.1:1080
```

### 5.2 使用SOCKS5代理的示例

```bash
# 全局SOCKS5代理配置
git config --global http.proxy socks5://127.0.0.1:1080
git config --global https.proxy socks5://127.0.0.1:1080
```

### 5.3 SSH配置文件示例（使用SOCKS5代理）

```
Host github.com
  HostName github.com
  User git
  ProxyCommand nc -x 127.0.0.1:1080 %h %p
```

## 6. 参考链接

- [Git官方文档 - 配置Git](https://git-scm.com/book/zh/v2/%E8%B5%B7%E6%AD%A5-%E9%85%8D%E7%BD%AE-Git)
- [SSH配置文件文档](https://man.openbsd.org/ssh_config)