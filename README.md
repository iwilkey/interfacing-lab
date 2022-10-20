# Flashing the Microcontroller...

* Make sure the microcontroller is plugged in via USB to host device.
* Open a terminal at repo head (../ECE-322/).
* Make sure your directory has a valid "solution.c" file with the main entry point.
* $ ./flash [directory-name]

Example: Flashing "Lab6" to the microcontroller. Assuming it is plugged in and all code in "Lab6/solution.c" is correct...

```console
iwilkey@Ians-MacBook-Air ECE-322 % ./flash Lab6
```