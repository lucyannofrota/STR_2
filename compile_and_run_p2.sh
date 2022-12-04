
#!/bin/sh

make
rm "P5_OUTPUT.txt"
# sudo ./problema_1 | tee "P1_OUTPUT.txt"
echo "Runing p5..."
sudo ./p5 >> "P5_OUTPUT.txt" 2>&1