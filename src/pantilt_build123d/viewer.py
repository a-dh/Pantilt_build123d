from build123d import Box
from ocp_vscode import show

def demo():
    return Box(30, 30, 30)

if __name__ == "__main__":
    part = demo()
    show(part)  # launches VS Code panel viewer if extension installed
