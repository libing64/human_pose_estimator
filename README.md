# Human Pose Estimator


## 1.1 Steps
* download trained models
* load models
* estimate human pose
* human pose drawing


![pose_skeleton](https://github.com/libing64/human_pose_estimator/blob/master/image/human_pose_estimator.gif)

## 1.2 Download models
```
cd catkin_ws/src/human_pose_estimator/models
git clone git@github.com:libing64/human_pose_estimator.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="human_pose_estimator"
```

## 1.3 Building
modify the model path in human_pose_estimator.h
```
cd catkin_ws/src
git clone git@github.com:libing64/human_pose_estimator.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="human_pose_estimator"
```


## 1.4 Running with usb camera
```
soure devel/setup.bash
roslaunch human_pose_estimator human_pose_estimator.launch
```

## 1.5 Test Environment
Ubuntu 20.04 + ros noetic


# 2. TODO
## 2.1 How to make the estimator more robust and accurate?
- [x] record screen to gif
- [ ] improve computation efficiency

## 3. record screen with byzanz
```
byzanz-record --delay 10 -d 30 human_pose_estimator.gif
```
