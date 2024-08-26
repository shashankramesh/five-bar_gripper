import os

def read_from_named_pipe(pipe_name):
    with open(pipe_name, 'r') as pipe:
        while True:
            line = pipe.readline()
            if not line:
                break
            print("Received data from C++:", line.strip())

if __name__ == "__main__":
    pipe_name = '/tmp/my_named_pipe'

    if not os.path.exists(pipe_name):
        print("Named pipe does not exist.")
        exit(1)

    read_from_named_pipe(pipe_name)

