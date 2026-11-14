# 实验数据

打开 dataset/overview.html 浏览三轴实验。ULog 由 Git LFS 存储，默认跳过下载。
从刚克隆的仓库根目录按需运行（需要 Git LFS）：

```bash
git lfs install --local
git lfs pull -I "data/experiment/pitch_pd_*/*.ulg" -X ""
```

只取 Roll 或 Yaw 时，将 pitch 换成 roll 或 yaw。下载全部日志：

```bash
git lfs pull -I "*.ulg" -X ""
```
