<div align="center">
    <h1>KISS-Matcher</h1>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://arxiv.org/abs/2409.15615"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
    <br />
    <br />
  <br />
  <br />
  <p align="center"><img src="https://github.com/user-attachments/assets/763bafef-c11a-4412-a9f7-f138fc12ff9f" alt="KISS Matcher" width="80%"/></p>
  <p><strong><em>Keep it simple, make it scalable.</em></strong></p>
  <p align="center">
    <strong>(May 19, 2026)</strong> pip installation is now live:
    <br/>
    <a href="https://pypi.org/project/kiss-matcher/"><img src="https://readme-typing-svg.demolab.com?background=0D1117&color=22C55E&font=Fira+Code&size=18&duration=2500&pause=800&center=true&vCenter=true&width=300&height=30&lines=%24+pip+install+kiss-matcher" alt="pip install kiss-matcher"/></a>
  </p>
</div>

______________________________________________________________________

# Py-KISS-Matcher

## Prerequisites

The prerequisites for Pybind11 are just the minimum requirements as follows:

```
pip3 install --upgrade pip setuptools wheel scikit-build-core ninja cmake build
```

## :package: Installation

And then, run the following command:

```
pip3 install -e .
```

If something goes weird, you can add `verbose` option like this:

```
pip3 install -e . --verbose
```

## :rocket: How to Run

### Example A. KITTI 10m Benchmark

TBU soon :)

### Example B. Registration from scan-level to map level

This examples are exactly same with the [C++ examples](https://github.com/MIT-SPARK/KISS-Matcher/tree/main/cpp/examples)!

First, download files via:

```
python3 utils/download_datasets.py
```

Then, all datas are downloaded in `data/` directory.

**Usage**: Run `examples/run_kiss_matcher.py` following template:

```
python3 examples/run_kiss_matcher.py \
    --src_path <src_pcd_file> \
    --tgt_path <tgt_pcd_file> \
    --resolution <resolution> \
    --yaw_aug_angle <yaw_aug_angle in deg (Optional)>
```

### Example B-0. Perform registration two point clouds from different viewpoints of Velodyne 16 at MIT campus

**Update (Sep 15, 2025):** I now use Viser for visualization because it is lighter than Open3D-based visualization.
To access it, simply open `http://localhost:8080` in your browser.

```
python3 examples/run_kiss_matcher.py \
    --src_path data/Vel16/src.pcd \
    --tgt_path data/Vel16/tgt.pcd \
    --resolution 0.2
```

**Result**

![Image](https://github.com/user-attachments/assets/c7d57fd1-24e7-458e-84a8-9b3578cc12dd)

### Example B-1. Perform registration two point clouds from different viewpoints of Velodyne 64 in KITTI dataset

```
python3 examples/run_kiss_matcher.py \
    --src_path data/Vel64/kitti_000540.pcd \
    --tgt_path data/Vel64/kitti_001319.pcd \
    --resolution 0.3
```

**Result**

![Image](https://github.com/user-attachments/assets/2adfa6d8-d3cb-4bd3-9283-26926f171806)

### Example B-2. Perform registration KITTI07 (Orange) and KITTI00 (Cyan) map clouds

```
python3 examples/run_kiss_matcher.py \
    --src_path data/KITTI00-to-07/kitti07.pcd \
    --tgt_path data/KITTI00-to-07/kitti00.pcd \
    --resolution 2.0
```

**Result**

![Image](https://github.com/user-attachments/assets/5716f629-19cc-4aa8-b715-70178cca8f20)

### Example B-3. Perform registration KITTI00 (Orange) and KITTI360-09 (Cyan) map clouds

```
python3 examples/run_kiss_matcher.py \
    --src_path data/KITTI00-to-KITTI360/kitti00.pcd \
    --tgt_path data/KITTI00-to-KITTI360/kitti360_09.pcd \
    --resolution 2.0
```

**Result**

![Image](https://github.com/user-attachments/assets/4e569bac-9264-457f-9e85-2664b3b76ed7)

### Example B-4. Perform registration heterogeneous LiDAR point cloud maps in `KAIST05` of the HeLiPR dataset

We're so excited in that initial transformation problem between Heterogeneous LiDAR SLAM now has been solved via KISS-Matcher!
By setting `<yaw_aug_angle>`, we can check whether it works even in the presence of huge pose discrepancy.

- Aeva-to-Livox

```
python3 examples/run_kiss_matcher.py \
    --src_path data/HeLiPR-KAIST05/Aeva.pcd \
    --tgt_path data/HeLiPR-KAIST05/Livox.pcd \
    --resolution 2.0 \
    --yaw_aug_angle 180
```

- Aeva-to-Ouster

```
python3 examples/run_kiss_matcher.py \
    --src_path data/HeLiPR-KAIST05/Aeva.pcd \
    --tgt_path data/HeLiPR-KAIST05/Ouster.pcd \
    --resolution 2.0 \
    --yaw_aug_angle 180
```

- Aeva-to-Ouster

```
python3 examples/run_kiss_matcher.py \
    --src_path data/HeLiPR-KAIST05/Livox.pcd \
    --tgt_path data/HeLiPR-KAIST05/Ouster.pcd \
    --resolution 2.0 \
    --yaw_aug_angle 180
```

**Result**

![Image](https://github.com/user-attachments/assets/9427e089-44f1-40e3-bc44-cc66f0c8e17c)

### Example B-5. Perform registration `Collosseo test0` (Orange) and `train0` (Cyan) sequence maps of the VBR dataset

In this example, there are non-negligible pitch and roll rotation. So, please set

```
python3 examples/run_kiss_matcher.py \
    --src_path data/VBR-Collosseo/test0.pcd \
    --tgt_path data/VBR-Collosseo/train0.pcd \
    --resolution 2.0
```

**Result**

![Image](https://github.com/user-attachments/assets/2e84608e-b1c8-4706-8ea9-f1149697cc4b)

> :warning: We have confirmed that the PCL visualizer triggers a segmentation fault when no GPU is available. In this case, all warped clouds are saved as ${SRC_NAME}\_warped.pcd, and we recommend using other visualization tools, such as CloudCompare, for visualization.

______________________________________________________________________

### Example C. TBU

______________________________________________________________________

## Citation

This study is the culmination of my continuous research since my graduate school years.
If you use this library for any academic work, please cite our original [paper](https://arxiv.org/abs/2409.15615), and refer to other papers that are highly relevant to KISS-Matcher.

<details>
  <summary><strong>See bibtex lists</a></strong></summary>

```bibtex
 @inproceedings{lim2025icra-KISSMatcher,
   title={{KISS-Matcher: Fast and Robust Point Cloud Registration Revisited}},
   author={Lim, Hyungtae and Kim, Daebeom and Shin, Gunhee and Shi, Jingnan and Vizzo, Ignacio and Myung, Hyun and Park, Jaesik and Carlone, Luca},
   booktitle={Proc. IEEE Int. Conf. Robot. Automat.},
   year={2025},
   codeurl   = {https://github.com/MIT-SPARK/KISS-Matcher},
   note   = {Accepted. To appear}
 }
```

```bibtex
 @article{Lim24ijrr-Quatropp,
   title={{Quatro++: R}obust global registration exploiting ground segmentation for loop closing in {LiDAR SLAM}},
   author={Lim, Hyungtae and Kim, Beomsoo and Kim, Daebeom and Mason Lee, Eungchang and Myung, Hyun},
   journal={Int. J. Robot. Res.},
   pages={685--715},
   year={2024},
   doi={10.1177/02783649231207654}
 }
```

```bibtex
@inproceedings{Lim22icra-Quatro,
    title={A single correspondence is enough: Robust global registration to avoid degeneracy in urban environments},
    author={Lim, Hyungtae and Yeon, Suyong and Ryu, Soohyun and Lee, Yonghan and Kim, Youngji and Yun, Jaeseong and Jung, Euigon and Lee, Donghwan and Myung, Hyun},
    booktitle={Proc. IEEE Int. Conf. Robot. Automat.},
    pages={8010--8017},
    year={2022}
}
```

```bibtex
@InProceedings{Shi21icra-robin,
    title={{ROBIN:} a Graph-Theoretic Approach to Reject Outliers in Robust Estimation using Invariants},
    author={J. Shi and H. Yang and L. Carlone},
    booktitle={Proc. IEEE Int. Conf. Robot. Automat.},
    note = {arXiv preprint: 2011.03659, \linkToPdf{https://arxiv.org/pdf/2011.03659.pdf}},
    pdf="https://arxiv.org/pdf/2011.03659.pdf",
    year={2021}
}
```

```bibtex
@article{Yang20tro-teaser,
    title={{TEASER: Fast and Certifiable Point Cloud Registration}},
    author={H. Yang and J. Shi and L. Carlone},
    journal={IEEE Trans. Robot.},
    volume = 37,
    number = 2,
    pages = {314--333},
    note = {extended arXiv version 2001.07715 \linkToPdf{https://arxiv.org/pdf/2001.07715.pdf}},
    pdf={https://arxiv.org/pdf/2001.07715.pdf},
    Year = {2020}
}
```

```bibtex
@inproceedings{Zhou16eccv-FGR,
    title={Fast global registration},
    fullauthor={Zhou, Qian-Yi and Park, Jaesik and Koltun, Vladlen},
    author={Q.Y. Zhou and J. Park and V. Koltun},
    booktitle={Proc. Eur. Conf. Comput. Vis.},
    pages={766--782},
    year={2016}
}
```

</details>
