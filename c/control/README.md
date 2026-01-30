# Compile the shared libraries

```sh
gcc -O2 -fPIC -shared \
    attitude_control.c \
    -o libcontrol.so \
    -lm
```

# Compile the test_main.c

```sh
gcc -Wall -Wextra -O2 \
    test_main.c attitude_control.c attitude_estimator.c \
    -lm \
    -o control_test
```
