To build this test code, first you need to install some libraries.

If you don't have brew installed, follow the directions at http://brew.sh/ to install it.

In Terminal, run

```
brew tap homebrew/versions
brew install glfw3
brew install glew
```

You can then compile this code using the Makefile, by running
```
make GUI=1
```
when in this folder.

Connect the Teensy/LIDAR to your computer and use the following command to get the name of the connected device.
```
ls /dev | grep tty.usbmodem
```

Run the code using the command below, replacing *number* with the number in the output from the command above.
```
./lidar_grapher dev /dev/tty.usbmodem<number>
```
