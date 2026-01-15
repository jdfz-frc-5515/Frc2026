Python 3.10.11

步骤 A：你作为开发者（导出清单）
在你的环境激活状态下，运行以下命令：

Bash

pip freeze > requirements.txt
这会生成一个 requirements.txt 文件。你需要把这个文件提交到 GitHub。

步骤 B：别人作为使用者（恢复环境）
当别人从 GitHub 拉取你的代码后，他们只需要执行以下“三部曲”：

创建自己的环境： python -m venv .venv

激活环境： (根据系统执行相应的激活命令)

根据清单安装：

Bash

pip install -r requirements.txt
执行完这一步，他们的环境就和你的一模一样了。