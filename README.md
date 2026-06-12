# 高超声速飞行器控制教程网页版

这是从原书项目中单独提取出来的网页站点版本，面向 GitHub Pages 直接部署。

## 目录

- `site/`：静态站点文件，直接作为 Pages 发布内容

## 发布方式

推荐使用 GitHub Actions 自动部署：

1. 把这个目录作为独立仓库推到 GitHub；
2. 在仓库设置里启用 GitHub Pages；
3. 使用仓库内的 Pages workflow 自动发布 `site/` 目录。

## 本地预览

如果只想本地查看，直接打开 `site/index.html` 即可。

## 更新内容

这个站点是静态成品。如果原书内容更新，先在原项目里重新生成网页，再把新的 `build/web/` 同步到这里的 `site/` 目录。
