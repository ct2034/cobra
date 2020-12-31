#/bin/bash
while read p; do
  sudo apt install -y $p
done <apt_dependencies.txt

cd COBRA
make
sudo ln -s $(pwd)/cobra /usr/bin/cobra
