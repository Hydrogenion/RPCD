## Quick Start
### Put your raw data in right place
Put your raw point cloud data in ```./data```

File structure should be like this
```bash
.
└── 2-l
    ├── 2-l - Cloud-body.txt
    ├── 2-l - Cloud-foot1.txt
    ├── 2-l - Cloud-foot2.txt
    ├── 2-l - Cloud-foot3.txt
    ├── 2-l - Cloud-foot4.txt
    ├── 2-l - Cloud-head.txt
    ├── 2-l - Cloud-tail.txt
    ├── 2-l-1.JPG
    ├── 2-l-2.JPG
    ├── 2-l-3.JPG
    ├── 2-l-4.JPG
    ├── 2-l-5.JPG
    ├── 2-l-6.JPG
    └── 2-l.ply
```

```2-l.ply``` is the raw point clouds

```*.txt``` is the parts' segmentation for annotations

```*.jpg``` is the different view of 3D object

**Please make sure the file extension is .ply->raw point clouds, .txt->segmentated point cloud**

### Preprocess the dataset
```python
python main.py -s ./data -d ./res
```
The result will be stored in ```./res```

File structure of ```./res```
```
.
└── 2-l
    ├── annotation.txt
    ├── clean_100k.ply
    ├── clean_10k.ply
    ├── clean_1k.ply
    ├── info.txt
    ├── mesh.ply
    ├── noiseless.ply
    ├── real_100k.ply
    ├── real_10k.ply
    └── real_1k.ply
```
```info.txt``` is basic info for this 3D shape

```annotation.txt``` is info for annotations

```clean_*k.ply``` is point cloud sampled from reconstructed mesh

```real_*k.ply``` is point cloud sampled from noiseless point cloud

```noiseless.ply``` is noiseless point cloud sampled from raw point cloud

#### Example for clean_100k
![Image](https://github-1253353217.cos.ap-beijing.myqcloud.com/clean_100k_eg.png)


### Show annotation
```python
python show_annotation.py -i ./res/2-l -t c # for clean_100k.ply
python show_annotation.py -i ./res/2-l -t r # for real_100k.ply
```

#### Example for annotation

![Image](https://github-1253353217.cos.ap-beijing.myqcloud.com/annotations.png)

## Visual Checker
You can use ```visual_checker.py``` to check your point cloud and mesh.
```python 
visual_checker.py -s res -d check.csv
```

程序使用说明:

        选中绘图窗口按快捷键：（关闭输入法）
        Q: 下一个
        A: 正常
        Z: 噪音过多 (手工难以剔除)
        X: 明显孔洞
        C: 车窗孔洞
        V: 明显缺失
        S: 切换分割视图 (有分割文件才可以)
        D: 分割标注错误 (如果仅是分割有孔洞，原模型没有孔洞，则记为分割标注错误，而非明显孔洞)
        W: 有离群点 (手工可以剔除)
        space: 切换背景 （绿色背景，更容易看出问题）
        .: 强制中断程序