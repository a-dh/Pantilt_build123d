from build123d import Box

def demo():
    # Simple demo part: 30x30x30 cube
    cube = Box(30, 30, 30)
    return cube

if __name__ == "__main__":
    part = demo()
    print("Demo part created:", part)
