
In the early development of convolutional neural networks (CNNs), convolutions with kernel size $(3\times 3)$, $(5\times 5)$, $(7\times 7)$ or even $(11\times 11)$are often used. In the more recent literature, however, $(1\times 1)$ convolutions are becoming prevalent. In this post, I will try to explain what $(1\times 1)$convolutions are and discuss why they are used in CNNs.

In the early development of convolutional neural networks (CNNs), convolutions with kernel size , or even are often used. In the more recent literature, however, convolutions are becoming prevalent. In this post, I will try to explain what convolutions are and discuss why they are used in CNNs.

In essence, convolutions are just convolutions with kernel size equal to , nothing new. Suppose the feature map size of one layer is , and it is followed by convolutions, and the output channel is . Then in this convolution layer, we will have filters, each of which has shape . The convolutional weight shape in this layer is , and the shape of bias terms is . The output feature map size after this convolutional layer is . So the spatial size of feature maps is preserved, but the number of feature maps are changed from 64 to .

The paper [_Network in network_](https://arxiv.org/abs/1312.4400) first proposed to use convolutions. In that paper, multiple convolutional layers is stacked together, which the authors call a _mlp layer_. Since the kernel size is , which can not learn a spatial relationship within channels, it is used solely to learn cross-channel relations of the feature maps. You can also think of it as a fully-connected operation, where for each feature map the multiplying coefficients are the same.

Later in the Google [Inception](https://arxiv.org/abs/1409.4842) paper, convolutions are also used in the inception module shown below.
![[Pasted image 20230717162118.png]]

The use of convolutions is mainly for dimension reduction in channels to reduce the computational cost of the following or convolutions. Another purpose is to increase the non-linearity as convolutions are immediately followed by ReLU. The two purposes of convolutions are stated clearly in this paper:

> That is, convolutions are used to compute reductions before the expensive and convolutions. Besides being used as reductions, they also include the use of rectified linear activation making them dual-purpose.

In the more recent [ResNet](https://arxiv.org/abs/1512.03385) paper, the authors proposed to use “bottleneck” structure (shown below), which also involves the use of convolutions. The purpose of it is also for reducing or increasing dimensions.
![[Pasted image 20230717162132.png]]

# Relationship between fully-connected layers and $1 \times 1$ convolutions
In image classification, researchers often use fixed size input image and fully connected layers as classifiers to classify the image. But according to the father of convolutional neural networks – Yann lecun, there is no such thing as fully-connected layers and you do not need to use fixed size input image in testing time:

> In Convolutional Nets, there is no such thing as “fully-connected layers”. There are only convolution layers with 1x1 convolution kernels and a full connection table. It’s a too-rarely-understood fact that ConvNets don’t need to have a fixed-size input. You can train them on inputs that happen to produce a single output vector (with no spatial extent), and then apply them to larger images. Instead of a single output vector, you then get a spatial map of output vectors. Each vector sees input windows at different locations on the input. In that scenario, the “fully connected layers” really act as 1x1 convolutions.

That is, we can interpret fully-connected layers as convolutions. For example, in VGG-16 network, the input image size is , the output is a classification score for 1000 classes, whose size is . If the input images have bigger size than , we can interpret the fully-connected layers as convolutions in a sliding windows fashion so that the output of the network is a classification map of size , where and . Each point in each classification map represent the probability of the corresponding receptive field in the original image belonging to that specific class.

In summary, convolutions are used for 2 purposes: ones is to reduce dimensions in order to reduce computational cost, the other is to increase non-linearity of the network (or the capacity of the network) in a cheap way.

*   [A discussion about the meaning of convolutions](https://groups.google.com/forum/#!topic/caffe-users/f1R-JrUQSMg)
*   [Why convolutions are used](https://stackoverflow.com/q/39366271/6064933)
*   [Another post about the meaning of convolutions](https://stats.stackexchange.com/q/194142/140049)
*   [A discussion about the significance of _network in network_](https://www.reddit.com/r/MachineLearning/comments/5n01i4/d_network_in_network_nin_is_it_still_useful/)
*   [How to interpret the fully-connected layers as convolutions](https://qr.ae/pG08uW).

Author jdhao

LastMod 2022-03-21

License [CC BY-NC-ND 4.0](https://creativecommons.org/licenses/by-nc-nd/4.0/)


[Source](https://jdhao.github.io/2017/09/29/1by1-convolution-in-cnn/)