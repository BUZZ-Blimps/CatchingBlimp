*!!! This steps should be executed on orangePi*

**Usage**: Run model and inference only on board without using PC

**1. Clone the repo**

```
git clone https://github.com/airockchip/rknn-toolkit2.git
```

**2. Install Miniconda on orangePi**

```
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
bash Miniconda3-latest-Linux-aarch64.sh
```

**3. Install the dependency (Using Miniconda)**

```
conda create -n rknn python=3.9
sudo apt-get update
sudo apt-get install -y python3 python3-dev python3-pip gcc
sudo apt-get install -y python3-opencv
sudo apt-get install -y python3-numpy
cd rknn-toolkit2/rknn-toolkit2-lite2/packages
pip3 install rknn_toolkit_lite2-x.y.z-cp39-cp39-linux_aarch64.whl
```

**4. Run the examples**

```
cd ~/rknn-toolkit2/rknn-toolkit-lite2/examples/resnet18
python3 test.py
```

**4. (Optional) Potential Errors**
- Not found librknnrt.so in /usr/lib\
   Solutions:
```
sudo mv librknnrt.so /usr/lib
```