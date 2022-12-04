
#!/bin/sh

make
rm "P1_OUTPUT.txt" "P2_OUTPUT.txt" "P3_OUTPUT.txt" "P4_OUTPUT.txt"
# sudo ./problema_1 | tee "P1_OUTPUT.txt"
echo "Runing p1..."
sudo ./p1 >> "P1_OUTPUT.txt" 2>&1
echo "Runing p2..."
sudo ./p2 >> "P2_OUTPUT.txt" 2>&1
echo "Runing p3..."
sudo ./p3 >> "P3_OUTPUT.txt" 2>&1
echo "Runing p4..."
sudo ./p4 >> "P4_OUTPUT.txt" 2>&1