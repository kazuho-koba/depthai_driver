import depthai as dai

print(dai.__version__)
for name in dir(dai.IMUSensor):
    if name.isupper():
        print(name)