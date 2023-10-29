import docker
client = docker.from_env()

dockerfile = "Dockerfile.humble.desktop"

image, streamer = client.images.build(
    path=".",
    dockerfile=dockerfile,
    tag="felix-humble-desktop:1.0"
)

for chunk in streamer:
    print("hello")
    if 'stream' in chunk:
        for line in chunk['stream'].splitlines():
            print(line)