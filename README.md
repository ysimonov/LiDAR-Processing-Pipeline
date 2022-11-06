# LiDAR-Processing-Pipeline
A complete processing pipeline for LiDAR on a moving vehicle.

### Outline
The goal is to develop a complete LiDAR pipeline that will perform the following steps:
* Point cloud downsampling / filtering (reducing number of points to be processed in consecutive steps of the pipeline)
* Ground segmentation (classifying points as ground (inliers) and non-ground (outliers))
* Obstacle clustering (splitting obstacle point clouds into separate groups based on Euclidean proximity)
* Polygonization (bounding Boxes with Quarternios describing object poses)
* Kalman filter tracking (persistently tracking object poses - Quarternions, w.r.t. global reference frame)

### Progress Tracking
* Downsampling (TODO)
* Segmentation (TODO)
* Clustering (TODO)
* Polygonization (TODO)
* Tracking (TODO)
