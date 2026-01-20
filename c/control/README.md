# Compile the shared libraries

```sh
gcc -O2 -fPIC -shared control.c -o libcontrol.so
```

# Compile the test_main.c

```sh
gcc -Wall -Wextra -O2 \
    test_main.c control.c \
    -lm \
    -o control_test
```
