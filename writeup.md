## Project: Perception Pick & Place

[//]: # (Image References)

[image1]: ./images/original.png
[image2]: ./images/no_outlier.png
[image3]: ./images/downsampled.png
[image4]: ./images/passthrough.png
[image5]: ./images/table.png
[image6]: ./images/objects.png
[image7]: ./images/cluster.png
[image8]: ./images/confusion.png
[image9]: ./images/labels1.png
[image10]: ./images/labels2.png
[image11]: ./images/lables3.png
[image12]: ./images/collision_map_table_2.png
[image13]: ./images/collision_map_full.png
[image14]: ./images/collision_map_picked.png
[image15]: ./images/pick_and_place.png

### Writeup
In this project I implemented the perception pipeline, the collision map construction and the pick and place robot task for the three test worlds. The following writeup is split into three tasks based on the rubric criteria, a section that discusses the pick and place challenge and a section that discusses details of the code implementation.

#### Task 1: Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
I constructed the perception pipeline by consecutively applying filtering, segmentation and detection algorithms in the `pcl_callback` method. The initial input to the perception pipeline is the output of an RGB-D camera which includes color and depth information making it possible to create a point cloud of points with color in 3D space. The perception algorithms in the pipeline operate on these point clouds. The original cloud looks like this

![original cloud][image1]

The first step in the pipeline is to clean up the initial point cloud, removing random noise using the statistical outlier filter. I obtained the parameters for the k-means and threshold by starting off with the values from the lesson and then through trial and error attempting to improve the result.

![outlier filtering][image2]

The second step is to perform voxel downsampling to reduce the number of points in the point cloud to make the algorithms run faster. 

![voxel downsampling][image3]

The next step is to limit the range along the z axis since only the points of interest lie in a limited region. After removing the table post RANSAC, I also limited the range along the y axis to be between -0.5 and 0.5 since all objects are within this range.

![passthrough filtering][image4]

As the fourth step, I applied the RANSAC algorithm which is a consensus based algorithm that can locate objects with known shape in the point cloud. The goal of this RANSAC is to locate plane surfaces, because we know this surface is the table. Once we know which points in the point cloud correspond to the table, we can remove these points and what is left over in the point cloud are the objects that we are interested in. The following images show the located table and the leftover point cloud points which are the objects.

![table][image5]
![objects][image6]

#### Task 2: Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.
The next step is to apply a segmentation algorithm to determine which point cloud points belong to the same object. For this, I used the Euclidean clustering algorithm. The main difficulty here was finding the correct parameters through trial and error. I received the best results using a cluster size between 50 and 2200 points and a cluster tolerance of 0.03. This means I will only continue adding points to the cluster if they are very close which seems to result in nicely split clusters. The result of the clustering, once the parameters are correct, is a cluster per object which I color with a different color per cluster.

![cluster][image7]

#### Task 3: Complete Exercise 3 Steps. Features extracted and SVM trained. Object recognition implemented.
Given the clusters of point clouds points, the robot needs to know which cluster is which object so that it can pick them up in the correct order from `pick_list.yaml`. To do this we train a Support Vector Machine using randomly generated poses of the objects we are looking for. This generation step is done by running `capture_features.py`. The generated features use the `compute_color_histograms` and `compute_normal_histograms` in `features.py` to generate a concatenated vector for every generated pose. These vectors are used to train the SVM and are stored in `trainig_set.sav`. The SVM is created from the training set using `train_svm.py` and stored in `model.sav`. 

I customized the feature generation, because the number of training samples generated was too low to give good results. I increased the number of samples per object to 100. Additionally, I computed the color histogram over the HSV (hue value saturation) space instead of RGB, which makes it more resilent to lighting changes. My training confusion matrix looks as follows:

![confusion matrix][image8]

From the confusion matrix it's obvious that my classifier has difficulty with `glue` objects. Many objects misclasify as clue, especially `book` objects. In fact, glue is the object that is not recognized correctly in `test3.world`. My classification results are as follows:

![labels 1][image9]
![labels 2][image10]
![labels 3][image11]

As can be seen from the images, in the first two `test.world` files I classify all objects correctly. In `test3.world` I classify 7/8 correctly. The only incorrect one is the glue, which seems to be due to the glue being behind other objects. When other objects are picked up, then the glue is classified correctly.

#### Pick and place challenge
Given the labeled objects, in the method `pr2_mover`, I implemented the mover by creating dictionary objects which return the point cloud and the centroid of the point cloud for a given label. In the code these are called `dict_cloud_by_label` and `dict_centroid_by_label`. Then I load in the pick list from the parameter server and iterate through the labels in the pick list. For each label I fetch the centroid and from it I compute the PR2 arm and the target box and I issue the `pick_place_routine` service request. In addition to sending the service requests I also store the message in the `output.yaml` files which are provided in the repository.

For the pick and place to work correctly, I additionally send a collision map to the PR2 robot. I compute this collision map in two steps. First inside the `__main__` part I start a left, right rotation of the PR2 robot so that I have collision data for the left and right sides of the table which are normally not visible. This rotation is implemented in the method `jointCheck`. The idea is that I have to keep a state machine which knows which rotation I need to do next. To determine whether a state transition needs to happen, I query the `/pr2/joint_states` topic. 

While the PR2 is rotating, it keeps updating a global list of table point cloud points called `table_cluster`. After the rotation is done, this list stores all the table points. Inside the method `pr2_mover` we then fill this list with the remaining objects on the table and send it to the topic `/pr2/3d_map/points` which is where the collision map is stored. I have the additional method `clear_collision_map()` which is needed because the collision map in PR2 is additive so if I want to make sure I am not sending data about already picked up object, I need to send a request to the `/clear_octomap` service, which will clear the exisiting map.

![collision map table][image12]

![collision map full][image13]

![collision map with objects where some have been picked up][image14]

![pick and place gazeebo][image15]

#### Implementation
I implemented everything as just one ROS node, using the `clustering` node from the project template. I noticed that topics sometimes will not receive messages if I publish a message immediately after creating a publisher, so I added a short timeout of two seconds implemented using `rospy.Rate`. 

As discussed in the pick and place section, I used a state machine implemented using arrays to setup the PR2 robot world joint rotation. This involves listening to a joint state topic and switching to a new state based on what is published on that topic. I found this approach quite ugly, but I couldn't find a better way to communicate with the PR2 to see if it finished moving. There was no ROS service that I could find.

The collision map calculation is implemented in a very expensive way and it only keeps track of the table and not the boxes. In the instructions it said to create a map for the table only which is what I did, however I think the correct thing to do is to also include the boxes in the collision map. It is expensive because I store the point cloud points all in a list which will contain many duplicates.

The pick and place action sometimes fails because the gripper does not attach correctly. I do not think it is due to the ROS node code. This happens almost every time with the "book" object.

Another problem with my implementation is that once an item has been picked up, the robot no longer knows the collision size of the picked-up item. This means it doesn't always manage to place them into the box because it doesn't know how it will fit. This seems to happen mainly with the large objects like the "snacks".


