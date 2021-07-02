#include <algorithm>
#include <cassert>
#include "BVH.hpp"
#include <map>

BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;

    time(&start);
    if (primitives.empty())
        return;
    root = recursiveBuild(primitives);
    // std::cout<<"primitives.size():"<<primitives.size()<<std::endl;
    // std::cout<<"root->left:"<<root->left<<std::endl;
    // std::cout<<"root->right:"<<root->right<<std::endl;

    time(&stop);

    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode *BVHAccel::recursiveBuild(std::vector<Object *> objects)
{
    BVHBuildNode *node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;

    bool sah = true;

    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1)
    {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2)
    {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();

        std::vector<Object *> leftshapes;
        std::vector<Object *> rightshapes;

        if (sah)
        {
            float nArea = centroidBounds.SurfaceArea();
            
            int minCostCoor = 0;
            int minCostIndex = 0;
            float minCost = std::numeric_limits<float>::infinity();
            std::map<int, std::map<int, int>> indexMap;

            for (int i = 0; i < 3; i++)
            {
                int bucketCount = 12;
                std::vector<Bounds3> boundsBucket;
                std::vector<int> countBucket;
                // initialize buckets
                for (int j = 0; j < bucketCount; j++)
                {
                    boundsBucket.push_back(Bounds3());
                    countBucket.push_back(0);
                }

                std::map<int, int> objMap;

                for (int j = 0; j < objects.size(); j++)
                {
                    int bid = bucketCount * (centroidBounds.Offset(objects[j]->getBounds().Centroid()))[i];
                    if (bid > bucketCount - 1)
                    {
                        bid = bucketCount - 1;
                    }
                    Bounds3 b = boundsBucket[bid];
                    b = Union(b, objects[j]->getBounds().Centroid());
                    boundsBucket[bid] = b;
                    countBucket[bid] = countBucket[bid] + 1;
                    objMap.insert(std::make_pair(j, bid));
                }

                for (int j = 1; j < boundsBucket.size(); j++)
                {
                    Bounds3 A;
                    Bounds3 B;
                    int countA = 0;
                    int countB = 0;
                    for (int k = 0; k < j; k++)
                    {
                        A = Union(A, boundsBucket[k]);
                        countA += countBucket[k];
                    }
                    for (int k = j; k < boundsBucket.size(); k++)
                    {
                        B = Union(B, boundsBucket[k]);
                        countB += countBucket[k];
                    }

                    float cost = 1 + (countA * A.SurfaceArea() + countB * B.SurfaceArea()) / nArea;

                    if (cost < minCost)
                    {
                        minCost = cost;
                        minCostIndex = j;
                        minCostCoor = i;
                    }
                }
            }
            for (int i = 0; i < objects.size(); i++)
            {
                if (indexMap[minCostCoor][i] < minCostIndex)
                {
                    leftshapes.push_back(objects[i]);
                }
                else
                {
                    rightshapes.push_back(objects[i]);
                }
            }
            // float min_cost = INFINITY;

            // // x axis
            // std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
            //     return f1->getBounds().Centroid().x <
            //            f2->getBounds().Centroid().x;
            // });
            // float range = centroidBounds.pMax.x - centroidBounds.pMin.x;
            // for (int b = 1; b < 31; ++b)
            // {
            //     float current_split_x = centroidBounds.pMin.x + b * range / 32;
            //     std::vector<Object*> left;
            //     std::vector<Object*> right;
            //     for (int i = 0; i < objects.size(); ++i)
            //     {
            //         if (objects[i]->getBounds().Centroid().x < current_split_x)
            //         {
            //                 left.push_back(objects[i]);
            //         }
            //         else
            //         {
            //                 for (int j = i; j < objects.size(); ++j)
            //                 {
            //                     right.push_back(objects[j]);
            //                 }
            //                 break;
            //         }
            //     }
            //     Bounds3 left_centroidBounds;
            //     Bounds3 right_centroidBounds;
            //     for (int i = 0; i < left.size(); ++i)
            //          left_centroidBounds =
            //             Union( left_centroidBounds, left[i]->getBounds().Centroid());
            //     for (int i = 0; i < right.size(); ++i)
            //          right_centroidBounds =
            //             Union( right_centroidBounds, right[i]->getBounds().Centroid());
            //     double leftSA = left_centroidBounds.SurfaceArea();
            //     double rightSA = right_centroidBounds.SurfaceArea();
            //     double currentSA = centroidBounds.SurfaceArea();
            // }
        }
        else
        {
            switch (dim)
            {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                          { return f1->getBounds().Centroid().x <
                                   f2->getBounds().Centroid().x; });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                          { return f1->getBounds().Centroid().y <
                                   f2->getBounds().Centroid().y; });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                          { return f1->getBounds().Centroid().z <
                                   f2->getBounds().Centroid().z; });
                break;
            }

            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            leftshapes = std::vector<Object *>(beginning, middling);
            rightshapes = std::vector<Object *>(middling, ending);
        }

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }
    return node;
}

Intersection BVHAccel::Intersect(const Ray &ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    // if (!root->left && !root->right)
    // {
    //     std::cout<<"root->left:"<<root->left<<std::endl;
    //     std::cout<<"root->right:"<<root->right<<std::endl;
    // }
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode *node, const Ray &ray) const
{
    // std::cout<<"node:"<< node <<std::endl;
    // std::cout<<"inside traverse node->left:"<< node->left <<std::endl;
    // std::cout<<"inside traverse node->right:"<< node->right <<std::endl;

    // TODO Traverse the BVH to find intersection

    Intersection inter;

    if (!node)
    {
        return inter;
    }
    if (!node->bounds.IntersectP(ray, ray.direction_inv))
    {
        return inter;
    }

    if (!node->left && !node->right)
    {
        inter = node->object->getIntersection(ray);
        return inter;
    }

    Intersection left = getIntersection(node->left, ray);
    Intersection right = getIntersection(node->right, ray);
    inter = left.distance < right.distance ? left : right;
    return inter;
}
