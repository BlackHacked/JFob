---
tags: AD
---
# Pointnet vanilla
![[Pasted image 20230912103630.png]]
Applying symmetric function to deal with the permutation problem(do not alter classification result when order of points change)
![[Pasted image 20230912103647.png]]
T-net is used to handle transformation variance, which means rotation of objects will not influence the predicted results.
## Pros and cons
### pros
- permutation invariant: points' order

### cons
The network always learns global features. Without localized contexts, it is not sensitive to repetitive features and not suitable for segmentation. Besides, it can not handle translation variance, since once the points' position changes, input data (x, y, z, i) will change and the global feature will change accordingly.
To sum up, limitations are listed below,
- not sensitive to repetitive features
- not good at segmentation
- can not handle translation variance

# Pointnet++
![[Pasted image 20230912155924.png]]


# PointPillars
Implementation of pointpillars based on Apollo platform[[apollo.canvas|apollo]][[JF.canvas|JF]]
