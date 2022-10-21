# ECE-322

This is a private repository for the Bradley University Electronic Interfacing Lab (ECE 322).

__Maintainer: Ian Wilkey (iwilkey)__

## Flashing a Microcontroller

* Make sure the microcontroller is plugged in via USB to host device.
* Open a terminal at repo head (../ECE-322/).
* Make sure your directory has a valid "solution.c" file with the main entry point.
* $ ./flash [directory-name]

### Example 
Let's flash "Lab6/" to the microcontroller. 

Assuming it is plugged in and all code in "Lab6/solution.c" is correct, run...
```console
iwilkey@Ians-MacBook-Air ECE-322 % ./flash Lab6
```
