# Add your user to the gpio group
sudo adduser $USER gpio

# Create a new udev rule
echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", PROGRAM="/bin/sh -c '\
     ''chown root:gpio /dev/gpiomem && chmod g+rw /dev/gpiomem'"' | \
sudo tee /etc/udev/rules.d/90-gpio.rules

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# You may need to log out and back in for the changes to take effect
