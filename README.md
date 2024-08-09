This is a record of the final project of ETH Advanced Computer Graphics cource. Only core source files and reports are uploaded.

## Introduction

### Inspirational Image

The topic of the competition this year is 'The more you look'. Which means we are encouraged to create visual masterpieces that attract viewers to uncover amazing detail.
![checkerborad](/images/MotivationalImage.png)

My idea is to create a checkerborad type image. The Inspirational Image is created by AI, but I wanted to render it manually to meet the topic.

### Features completed

- Path Tracing Intergrator
  
  - Direct Illumination: Emitter sampling, Material sampling, Multiple importance sampling
  - Global Illumination: Material sampling, Multiple importance sampling
- Photon mapping Intergrator
- Modeling mesh
- Microfacet BRDF sampling
- Dielectric BSDF
- Object instancing
- Addictional emitter:
  
  - Point / Area / Distant light
  - Environment map emitter
- MipMaping
- Depth of Field

## Implementation & Validation

### Path Tracing Intergrators

#### Direct Illumination:

|      | odyssey | veach |
| ---- | ------- | ----- |
| ems  |    ![ems](/reports/homework-3/images/odyssey_ems.png)     |  ![ems](/reports/homework-3/images/veach_ems.png)       |
| mats |   ![mats](/reports/homework-3/images/odyssey_mats.png)      |    ![mats](/reports/homework-3/images/veach_mats.png)     |
| mis  |   ![vis](/reports/homework-3/images/odyssey_mis.png)      |  ![vis](/reports/homework-3/images/veach_mis.png)      |

#### Global Illumination:

|      | cbox| cgltri | table |
| ---- | ------- | ----- | --- |
| mats |    ![mats](/reports/homework-4/images/cbox_path_mats.png)     |   ![mats](/reports/homework-4/images/cgltri_path_mats.png)         |  ![mats](/reports/homework-4/images/table_path_mats.png)  |
| mis |    ![mis](/reports/homework-4/images/cbox_path_mis.png)     |   ![mis](/reports/homework-4/images/cgltri_path_mis.png)         |  ![mis](/reports/homework-4/images/table_path_mis.png)  |
| pmap|   ![pmap](/reports/homework-4/images/cbox_pmap.png)     |   ![pmap](/reports/homework-4/images/cgltri_pmap.png)         | ![pmap](/reports/homework-4/images/table_pmap.png)  |




### Modeling mesh

To achieve the sense of order and coincidence in a picture, we need to select proper objects and place them in the correct direction and position. And in some of the cases, we need to modified a little bit on the objects to generate a perfect division in vision.

I have been working in blender by doing translating, rotating, scaling, and model modification on models, camera, and the light source in the scene, to make the whole picture to achieve the result that we expected under the effect of light and shadow.

I also build a grid framework to help me adjusting object in the camera view.

**Result:**

|                                    |                                  |
| ---------------------------------- | -------------------------------- |
| ![300](/images/model_overview.png) | ![300](/images/model_camera.png) |
| ![300](/images/model_modeling.png) | ![300](/images/model_mirror.png) |

**Remark:**
the sence generated in blender can be export to mitusba3 by applying add-on [Mitsuba-Blender](https://github.com/mitsuba-renderer/mitsuba-blender), to Nori by applying add-on [Nori-Blender](https://github.com/wjakob/nori/tree/master/ext/plugin)

### Depth of Fields

File added

```
src/realistic.cpp
```

File touched

```
src/path_mis.cpp
```

In realistic.cpp, we need to sample point on the 'lens', then use Gaussian lens equation to compute the focus point where sampled ray and central ray intersect, and finally to get the sampled ray from the sampled point on 'lens'.

|                                                               |                                                                    |
| ------------------------------------------------------------- | ------------------------------------------------------------------ |
| ![300](/images/small.png)aperture = 0.1 | ![300](/images/normalCamera.png)aperture = 1 |
| ![300](/images/big.png)aperture = 2     | ![300](/images/superbig.png)aperture = 3     |

Note that the focal length are fixed in the image above.

### Emitter: Distant Light & Environment Map Emitter

File added

```
src/envlight.cpp
src/distantlight.cpp
```

File touched

```
CMakeLists.txt
include/nori/emitter.h
src/path_mis.cpp
```

According to the book pbrtv3, these two emitters are infinite emitters. They are emitting light from a super far distant as identify. And they don't have solid objects exist in the scene. So when calculating their light contribution in intergrator, we need to go in another way, because it is impossible for the path tracing light to hit these emitter without entities. And these emitters need to provid their special eval method.

**emitter.h**

Add Le() method to be overrided in infinite emitters subclass.
distantlight.cpp

All the light can be seem as parallel since they come from far far away(like sunlight). So the init parameter would be irradience and direction.

when being sampled, return m_radiance and set light m_directoin directly.

And the eval() function should return the intergration of the light irradience witin the scene bounding area.

Beacuse they are all Delta lights, pdf() return 1.0 .

Distant light contribution can be collect as a random emitter in getRandomEmitter() in emitter sampling part, and simply contribute its radience by calling sample().

**Results:**

![300](/images/distant_nori.png)

<p align='center'>light from left</p>

![300](/images/distant_left.png)

<p align='center'>light from right</p>

**envlight.cpp**

A kind of light is an infinitely far-away area light source that surrounds the entire scene. It support user to attach environment texture on it.

The way it sampling light is performing a pipeline from sampling point in uv map into the light direction that hit the scene.

To do sampling in uvmap we need to compute the sample weight of each pixel on map, so the first work is to precompute the PDF map of the uvMap by the radience that the pixel carries. Then we can perform weighted sampling on the uvMap, to achieve the effect of the brighter a area is, the more possible the area would emitting light.(like the light source in the environment)

Then perform the transformation from uv Coordinate, to sphere Coordinate then to the query direction in world coordinate. And return the interpolated radience(color) in the vuMap with corresponding pdf.

**Results:**
![300](/images/evn_nori.png)

**path_mis.cpp**

Because the light would never hit these two emitters, so the old intergrator will never have mats term contribution, and the part that doesn't hit any object would remain black.

So I add two way to fit these emitters. One is when the tracing ray doesn't hit any object and escape the scene, it can be seem that it hit the covering environment emitter shpere. We need to query the color with the escaping ray Rec in the new Emitter method Le(Rec)

And when doing Emitter sampling, these emitter should be seem as normal emitters to being smapled, so we just need to make sure the sample() result is correct.

**Results:**
![300](/images/multilight_nori.png)

<p align='center'>Combaniation of Distance light & sky environment light</p>

### MipMaping

File added

```
src/mipmaptexture.cpp
```

File touched

```
include/nori/ray.h
include/nori/shape.h
src/mipmaptexture.cpp
src/path_mis.cpp
src/mesh.cpp
src/perspective.cpp
```

**mipmaptexture.cpp**

According to the book pbrtv3, mipMap should be intergrated in imageTexure, and the new class be able generate all level of mipmaps and provide interface to query color in different level mipmap.

So I write a generateMipmap() function and will be call when a imageFile is loaded.

It calculate the max level by log2(MaxResolusion), and create higher level of Mipmap with half of resolution compared to the lower level both in height and width, by averaging the 4 neighbor pixel color into one pixel to the next level.

When being query, it take the differential value L to culculate the Mipmap level and perform **trilinear interpolation** to compute the color between level and level+1.

|         |                            |
| ------- | -------------------------- |
| Level0  | ![0](/images/Level_0.jpg)  |
| Level1  | ![0](/images/Level_1.jpg)  |
| Level2  | ![0](/images/Level_2.jpg)  |
| Level3  | ![0](/images/Level_3.jpg)  |
| Level4  | ![0](/images/Level_4.jpg)  |
| Level5  | ![0](/images/Level_5.jpg)  |
| Level6  | ![0](/images/Level_6.jpg)  |
| Level7  | ![0](/images/Level_7.jpg)  |
| Level8  | ![0](/images/Level_8.jpg)  |
| Level9  | ![0](/images/Level_9.jpg)  |
| Level10 | ![0](/images/Level_10.jpg) |

**ray.h**

Add rayDifferential variable as pbrtv3 to ray Class.

**perspective.cpp**

Add genertateRayDifferential as pbrtv3 introduce. The RayDifferentials are generated by finding the Raydifferential in 1px offset in x and y direction of the camare film. Need to perform the transformation from clip coord to camara coord then to world coord.

**shape.h, mesh.cpp**

Add RayDifferentials related variables in Interscetion structure to record differentials along uv, xy, on hitting point p.
When mesh being hit, compute the differential value(changing rate from clip coord to world coord) in setHitInfomation().

**path_mis.cpp**

Pass the differential value inside Interscetion into the mipMap query to get the color.

![300](/images/high_imagemapping.png)

<p align='center'>normal secne</p>

![300](/images/high_mipmapping.png)

<p align='center'>Applying Mipmapping</p>

![300](/images/mipmapping.png)

<p align='center'>Level_distribution</p>

### Object Instancing

File added

```
include/nori/instanct.h
include/nori/reference.h
src/reference.cpp
src/instance.cpp
```

File touched

```
include/nori/shape.h
src/scene.cpp
```

According to the lecture I decided to create two new subclass of shape, representing reference and instance thus we can directly add them into xml. And the instance object only carries transformation from local(reference) to the world, which can be set within xml.

All the intersections to be happen with this type of shape as well as the intersecections recording is processd is in a local(reference) coordinate.

**reference.h & reference.cpp**

A simple copy of shape, to store reference object and local infomation.

**instance.h & instance.cpp**

The class representing instance. The shape-methods are overrided, to map the ray into local coordinate, and setHitInfo in local coordinat. After that turn the HitInfo back to the world coordinat.

**scene.cpp & shape.h**

Modified and added some functions to support the instancing system.

**Result:**

|                                                                 |                                                                 |
| --------------------------------------------------------------- | --------------------------------------------------------------- |
| ![300](/images/noInstance.png)origin ball | ![300](/images/Instance.png)with instance |

## Final Image

![fishtank](/images/final.png)

