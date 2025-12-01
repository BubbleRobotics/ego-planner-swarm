#include "maps.hpp"

#include <algorithm>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "perlinnoise.hpp"

using namespace mocka;

// Generate a random map
void
Maps::randomMapGenerate()
{

  std::default_random_engine eng(info.seed);

  double _resolution = 1 / info.scale;

  // Set map bounds
  double _x_l = -info.sizeX / (2 * info.scale);
  double _x_h = info.sizeX / (2 * info.scale);
  double _y_l = -info.sizeY / (2 * info.scale);
  double _y_h = info.sizeY / (2 * info.scale);
  double _h_l = 0;
  double _h_h = info.sizeZ / info.scale;

  // Get parameters
  double _w_l, _w_h;
  int    _ObsNum;

  info.node->declare_parameter("width_min", 0.6);
  info.node->declare_parameter("width_max", 1.5);
  info.node->declare_parameter("obstacle_number", 10);

  info.node->get_parameter("width_min", _w_l);
  info.node->get_parameter("width_max", _w_h);
  info.node->get_parameter("obstacle_number", _ObsNum);

  // Set random distributions
  std::uniform_real_distribution<double> rand_x;
  std::uniform_real_distribution<double> rand_y;
  std::uniform_real_distribution<double> rand_w;
  std::uniform_real_distribution<double> rand_h;

  pcl::PointXYZ pt_random;

  rand_x = std::uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = std::uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = std::uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = std::uniform_real_distribution<double>(_h_l, _h_h);

  // Generate random obstacles
  for (int i = 0; i < _ObsNum; i++)
  {
    double x, y;
    x = rand_x(eng);
    y = rand_y(eng);

    double w, h;
    w = rand_w(eng);
    h = rand_h(eng);

    int widNum = ceil(w / _resolution);
    int heiNum = ceil(h / _resolution);

    // Set obstacle bounds (in grid indices)
    int rl, rh, sl, sh;
    rl = -widNum / 2;
    rh = widNum / 2;
    sl = -widNum / 2;
    sh = widNum / 2;

    // Fill obstacle point cloud
    for (int r = rl; r < rh; r++)
      for (int s = sl; s < sh; s++)
      {
        for (int t = 0; t < heiNum; t++)
        {
          // Only keep boundary voxels (forming obstacle surfaces)
          if ((r - rl) * (r - rh + 1) * (s - sl) * (s - sh + 1) * t *
                (t - heiNum + 1) ==
              0)
          {
            pt_random.x = x + r * _resolution;
            pt_random.y = y + s * _resolution;
            pt_random.z = t * _resolution;
            info.cloud->points.push_back(pt_random);
          }
        }
      }
  }

  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;

  pcl2ros();
}

// Convert PCL point cloud to ROS message
void
Maps::pcl2ros()
{
    pcl::toROSMsg(*info.cloud, *info.output);
    info.output->header.frame_id = "ego_world";
    RCLCPP_INFO(info.node->get_logger(), "finish: infill %lf%%",
                info.cloud->width / (1.0 * info.sizeX * info.sizeY * info.sizeZ));
}

// Use Perlin noise to generate 3D voxel map data.
// By controlling complexity, fill rate, and fractal layers, it creates
// terrain-like obstacle distributions suitable for simulation environments.
void
Maps::perlin3D()
{
  double complexity;   // Complexity of the noise (frequency scaling)
  double fill;         // Fill ratio (percentage of occupied voxels)
  int    fractal;      // Number of fractal layers
  double attenuation;  // Attenuation factor per layer

  info.node->declare_parameter("complexity", 0.142857);
  info.node->declare_parameter("fill", 0.38);
  info.node->declare_parameter("fractal", 1);
  info.node->declare_parameter("attenuation", 0.5);

  // Get parameter values
  info.node->get_parameter("complexity", complexity);
  info.node->get_parameter("fill", fill);
  info.node->get_parameter("fractal", fractal);
  info.node->get_parameter("attenuation", attenuation);

  info.cloud->width  = info.sizeX * info.sizeY * info.sizeZ;
  info.cloud->height = 1;
  info.cloud->points.resize(info.cloud->width * info.cloud->height);

  PerlinNoise noise(info.seed);

  // Generate noise values and sort them
  std::vector<double>* v = new std::vector<double>;
  v->reserve(info.cloud->width);
  for (int i = 0; i < info.sizeX; ++i)
  {
    for (int j = 0; j < info.sizeY; ++j)
    {
      for (int k = 0; k < info.sizeZ; ++k)
      {
        double tnoise = 0;
        for (int it = 1; it <= fractal; ++it)
        {
          int    dfv = pow(2, it);
          double ta  = attenuation / it;
          tnoise += ta * noise.noise(dfv * i * complexity,
                                     dfv * j * complexity,
                                     dfv * k * complexity);
        }
        v->push_back(tnoise);
      }
    }
  }
  // Sort and compute threshold value based on fill ratio
  std::sort(v->begin(), v->end());
  int    tpos = info.cloud->width * (1 - fill);
  double tmp  = v->at(tpos);
  RCLCPP_INFO(rclcpp::get_logger("perlin3D"), "threshold: %lf", tmp);

  // Generate point cloud according to threshold
  int pos = 0;
  for (int i = 0; i < info.sizeX; ++i)
  {
    for (int j = 0; j < info.sizeY; ++j)
    {
      for (int k = 0; k < info.sizeZ; ++k)
      {
        double tnoise = 0;
        for (int it = 1; it <= fractal; ++it)
        {
          int    dfv = pow(2, it);
          double ta  = attenuation / it;
          tnoise += ta * noise.noise(dfv * i * complexity,
                                     dfv * j * complexity,
                                     dfv * k * complexity);
        }
        if (tnoise > tmp)
        {
          info.cloud->points[pos].x =
            i / info.scale - info.sizeX / (2 * info.scale);
          info.cloud->points[pos].y =
            j / info.scale - info.sizeY / (2 * info.scale);
          info.cloud->points[pos].z = k / info.scale;
          pos++;
        }
      }
    }
  }
  // Update point cloud width and shrink memory
  info.cloud->width = pos;
  RCLCPP_INFO(rclcpp::get_logger("perlin3D"), "the number of points before optimization is %d", info.cloud->width);
  info.cloud->points.resize(info.cloud->width * info.cloud->height);
  pcl2ros();
}

// Generate a maze using the recursive division method
void
Maps::recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi& maze)
{
  RCLCPP_INFO(rclcpp::get_logger("recursiveDivision"), "generating maze with width %d , height %d", xh - xl + 1, yh - yl + 1);

  // If the current region is at least 5x5, generate a vertical and a horizontal
  // wall to divide it into 4 subregions, then recursively process each one.
  if (xl < xh - 3 && yl < yh - 3)
  { // the remaining area is larger than or equal to 5*5, need to add both x
    // wall and y wall
    bool valid = false; // used to judge whether the wall selection is valid
    int  xm    = 0;
    int  ym    = 0;
    RCLCPP_INFO(rclcpp::get_logger("recursiveDivision"), "entered 5*5 mode");
    // Generate center position of the walls
    while (valid == false)
    {
      xm = (std::rand() % (xh - xl - 1) + xl +
            1); // random number between xl+1 and xh-1 (no sense to add a wall on the boundary)
      ym = (std::rand() % (yh - yl - 1) + yl +
            1); // random number between yl+1 and yh-1 (no sense to add a wall on the boundary)
      if (xl - 1 >= 0)
      { // there is a point at xl-1,ym
        if (maze(xl - 1, ym) == 0)
        { // this is an opening, need to change random number
          continue;
        }
      }

      else if (xh + 1 <= maze.cols() - 1)
      { // there is a point at xh+1,ym
        if (maze(xh + 1, ym) == 0)
        { // this is an opening, need to change random number
          continue;
        }
      }

      else if (yl - 1 >= 0)
      { // there is a point at xm,yl-1
        if (maze(xm, yl - 1) == 0)
        { // this is an opening, need to change random number
          continue;
        }
      }

      else if (yh + 1 <= maze.rows() - 1)
      { // there is a point at xm,yh+1
        if (maze(xm, yh + 1) == 0)
        { // this is an opening, need to change random number
          continue;
        }
      }

      valid = true;

    } // xm and ym are now the valid coordinates of the center of the walls
    // Add vertical and horizontal walls
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    // Randomly generate door positions
    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell

    // If the new walls block pre-existing passages at the boundaries, reopen them
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
      }
    }

    std::cout << maze << std::endl;
    // Recursively divide the four subregions
    recursiveDivision(xl, xm - 1, yl, ym - 1, maze);
    recursiveDivision(xm + 1, xh, yl, ym - 1, maze);
    recursiveDivision(xl, xm - 1, ym + 1, yh, maze);
    recursiveDivision(xm + 1, xh, ym + 1, yh, maze);

    RCLCPP_INFO(rclcpp::get_logger("recursiveDivision"), "finished generating maze with width %d , height %d",
                xh - xl + 1,
                yh - yl + 1);
    std::cout << maze << std::endl;
    return;
  } // when the remaining area is larger than or equal to 5*5

  // Special handling for smaller regions (but still reasonably big)
  else if (xl < xh - 2 && yl < yh - 2)
  {
    // bool valid     = false; // used to judge whether the wall selection is valid
    int  xm        = 0;
    int  ym        = 0;
    int  doorcount = 0;
    xm             = (std::rand() % (xh - xl - 1) + xl +
          1); // random number between xl+1 and xh-1
    ym =
      (std::rand() % (yh - yl - 1) + yl +
       1); // random number between yl+1 and yh-1
           // xm and ym are now the valid coordinates of the center of the walls
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
        doorcount++;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
        doorcount++;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
        doorcount++;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
        doorcount++;
      }
    }

    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell
    std::cout << maze << std::endl;

    RCLCPP_INFO(rclcpp::get_logger("recursiveDivision"), "finished generating maze with width %d , height %d",
            xh - xl + 1,
            yh - yl + 1);
    std::cout << maze << std::endl;
    return;
  }

  else if (xl < xh - 1 && yl < yh - 2)
  { // the case of 3*4+
    RCLCPP_INFO(rclcpp::get_logger("recursiveDivision"), "entered 3*4+ mode");
    int doorcount = 0;
    int ym        = 0;
    for (int i = yl; i <= yh; i++)
    {
      maze(xl + 1, i) = 1;
    } // filling a center wall
    if (yl - 1 >= 0)
    {
      if (maze(xl + 1, yl - 1) == 0)
      {
        maze(xl + 1, yl) = 0;
        doorcount++;
      }
    }
    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xl + 1, yh + 1) == 0)
      {
        maze(xl + 1, yh) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    if (doorcount == 0)
    {
      ym               = std::rand() % (yh - yl + 1) + yl;
      maze(xl + 1, ym) = 0;
    }
  } // the case of 4+*3
  //
  else if (xl < xh - 2 && yl < yh - 1)
  { // the case of 4+*3
    RCLCPP_INFO(rclcpp::get_logger("recursiveDivision"), "entered 4+*3 mode");
    int doorcount = 0;
    int xm        = 0;
    for (int i = xl; i <= xh; i++)
    {
      maze(i, yl + 1) = 1;
    } // filling a center wall
    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, yl + 1) == 0)
      {
        maze(xl, yl + 1) = 0;
        doorcount++;
      }
    }
    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, yl + 1) == 0)
      {
        maze(xh, yl + 1) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    if (doorcount == 0)
    {
      xm               = std::rand() % (xh - xl + 1) + xl;
      maze(xm, yl + 1) = 0;
    }
  } // the case of 4+*3

  else if (xl < xh - 1 && yl < yh - 1)
  { // the case of 3*3
    maze(xl + 1, yl + 1) = 1;
    return;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("recursiveDivision"), "finished generating maze with width %d , height %d",
                xh - xl + 1,
                yh - yl + 1);
    return;
  }
}

// Generate maze using recursive subdivision (alternative implementation)
void
Maps::recursizeDivisionMaze(Eigen::MatrixXi& maze)
{
  //! @todo all bugs here...
  int sx = maze.rows();
  int sy = maze.cols();

  int px, py;

  if (sx > 5)
    px = (std::rand() % (sx - 3) + 1);
  else
    return;

  if (sy > 5)
    py = (std::rand() % (sy - 3) + 1);
  else
    return;

  RCLCPP_INFO(rclcpp::get_logger("recursiveDivisionMaze"), "debug %d %d %d %d", sx, sy, px, py);

  int x1, x2, y1, y2;

  if (px != 1)
    x1 = (std::rand() % (px - 1) + 1);
  else
    x1 = 1;

  if ((sx - px - 3) > 0)
    x2 = (std::rand() % (sx - px - 3) + px + 1);
  else
    x2 = px + 1;

  if (py != 1)
    y1 = (std::rand() % (py - 1) + 1);
  else
    y1 = 1;

  if ((sy - py - 3) > 0)
    y2 = (std::rand() % (sy - py - 3) + py + 1);
  else
    y2 = py + 1;
  RCLCPP_INFO(rclcpp::get_logger("recursiveDivisionMaze"), "%d %d %d %d", x1, x2, y1, y2);

  if (px != 1 && px != (sx - 2))
  {
    for (int i = 1; i < (sy - 1); ++i)
    {
      if (i != y1 && i != y2)
        maze(px, i) = 1;
    }
  }
  if (py != 1 && py != (sy - 2))
  {
    for (int i = 1; i < (sx - 1); ++i)
    {
      if (i != x1 && i != x2)
        maze(i, py) = 1;
    }
  }
  switch (std::rand() % 4)
  {
    case 0:
      maze(x1, py) = 1;
      break;
    case 1:
      maze(x2, py) = 1;
      break;
    case 2:
      maze(px, y1) = 1;
      break;
    case 3:
      maze(px, y2) = 1;
      break;
  }

  if (px > 2 && py > 2)
  {
    Eigen::MatrixXi sub = maze.block(0, 0, px + 1, py + 1);
    recursizeDivisionMaze(sub);
    maze.block(0, 0, px, py) = sub;
  }
  if (px > 2 && (sy - py - 1) > 2)
  {
    Eigen::MatrixXi sub = maze.block(0, py, px + 1, sy - py);
    recursizeDivisionMaze(sub);
    maze.block(0, py, px + 1, sy - py) = sub;
  }
  if (py > 2 && (sx - px - 1) > 2)
  {
    Eigen::MatrixXi sub = maze.block(px, 0, sx - px, py + 1);
    recursizeDivisionMaze(sub);
    maze.block(px, 0, sx - px, py + 1) = sub;
  }
  if ((sx - px - 1) > 2 && (sy - py - 1) > 2)
  {

    Eigen::MatrixXi sub = maze.block(px, py, sy - px, sy - py);

    recursizeDivisionMaze(sub);
    maze.block(px, py, sy - px, sy - py) = sub;
  }
}

// Generate a 2D maze and convert it to a 3D point cloud
void
Maps::maze2D()
{
  double width;
  int    type;
  int    addWallX;
  int    addWallY;
  info.node->declare_parameter("road_width", 1.0);
  info.node->declare_parameter("add_wall_x", 0);
  info.node->declare_parameter("add_wall_y", 0);
  info.node->declare_parameter("maze_type", 1);

  info.node->get_parameter("road_width", width);
  info.node->get_parameter("add_wall_x", addWallX);
  info.node->get_parameter("add_wall_y", addWallY);
  info.node->get_parameter("maze_type", type);

  // Compute maze rows/cols and initialize matrix
  int mx = info.sizeX / (width * info.scale);
  int my = info.sizeY / (width * info.scale);

  Eigen::MatrixXi maze(mx, my);
  maze.setZero();

  // Generate maze with recursive division
  switch (type)
  {
    case 1:
      recursiveDivision(0, maze.cols() - 1, 0, maze.rows() - 1, maze);
      break;
  }

  // Add boundary walls in X direction if needed
  if (addWallX)
  {
    for (int i = 0; i < mx; ++i)
    {
      maze(i, 0)      = 1;
      maze(i, my - 1) = 1;
    }
  }
  // Add boundary walls in Y direction if needed
  if (addWallY)
  {
    for (int i = 0; i < my; ++i)
    {
      maze(0, i)      = 1;
      maze(mx - 1, i) = 1;
    }
  }

  std::cout << maze << std::endl;

  // Convert maze matrix into a 3D point cloud (extruding walls in Z)
  for (int i = 0; i < mx; ++i)
  {
    for (int j = 0; j < my; ++j)
    {
      if (maze(i, j))
      {
        for (int ii = 0; ii < width * info.scale; ++ii)
        {
          for (int jj = 0; jj < width * info.scale; ++jj)
          {
            for (int k = 0; k < info.sizeZ; ++k)
            {
              pcl::PointXYZ pt_random;
              pt_random.x =
                i * width + ii / info.scale - info.sizeX / (2.0 * info.scale);
              pt_random.y =
                j * width + jj / info.scale - info.sizeY / (2.0 * info.scale);
              pt_random.z = k / info.scale;
              info.cloud->points.push_back(pt_random);
            }
          }
        }
      }
    }
  }
  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;
  pcl2ros();
}

Maps::BasicInfo
Maps::getInfo() const
{
  return info;
}

void
Maps::setInfo(const BasicInfo& value)
{
  info = value;
}

Maps::Maps()
{
}

void
Maps::generate(int type)
{
  switch (type)
  {
    default:
    case 1:
      perlin3D();
      break;
    case 2:
      randomMapGenerate();
      break;
    case 3:
      std::srand(info.seed);
      maze2D();
      break;
    case 4: // generating 3D maze
      std::srand(info.seed);
      Maze3DGen();
      break;
  }
}

pcl::PointXYZ
MazePoint::getPoint()
{
  return point;
}

int
MazePoint::getPoint1()
{
  return point1;
}

int
MazePoint::getPoint2()
{
  return point2;
}

double
MazePoint::getDist1()
{
  return dist1;
}

double
MazePoint::getDist2()
{
  return dist2;
}

void
MazePoint::setPoint(pcl::PointXYZ p)
{
  point = p;
}

void
MazePoint::setPoint1(int p)
{
  point1 = p;
}

void
MazePoint::setPoint2(int p)
{
  point2 = p;
}

void
MazePoint::setDist1(double set)
{
  dist1 = set;
}

void
MazePoint::setDist2(double set)
{
  dist2 = set;
}

void
Maps::Maze3DGen()
{
  // Get required parameters from the ROS node
  int    numNodes;
  double connectivity;
  int    nodeRad;
  int    roadRad;

  // Declare parameters and default values
  info.node->declare_parameter("numNodes", 10);
  info.node->declare_parameter("connectivity", 0.5);
  info.node->declare_parameter("nodeRad", 3);
  info.node->declare_parameter("roadRad", 2);

  // Get parameter values
  info.node->get_parameter("numNodes", numNodes);
  info.node->get_parameter("connectivity", connectivity);
  info.node->get_parameter("nodeRad", nodeRad);
  info.node->get_parameter("roadRad", roadRad);

  // Log received parameters
  RCLCPP_INFO(info.node->get_logger(), 
              "received parameters : numNodes: %d connectivity: %f nodeRad: %d roadRad: %d",
              numNodes, connectivity, nodeRad, roadRad);
  // Generating random core points
  std::vector<pcl::PointXYZ> base;

  for (int i = 0; i < numNodes; i++)
  {
    double rx = std::rand() / RAND_MAX +
                (std::rand() % info.sizeX) / info.scale -
                info.sizeX / (2 * info.scale);
    double ry = std::rand() / RAND_MAX +
                (std::rand() % info.sizeY) / info.scale -
                info.sizeY / (2 * info.scale);
    double rz = std::rand() / RAND_MAX +
                (std::rand() % info.sizeZ) / info.scale -
                info.sizeZ / (2 * info.scale);
    RCLCPP_INFO(rclcpp::get_logger("Maze3DGen"), "point: x: %f , y: %f , z: %f", rx, ry, rz);

    pcl::PointXYZ pt_random;
    pt_random.x = rx;
    pt_random.y = ry;
    pt_random.z = rz;
    base.push_back(pt_random);
  } // generating random cores in the space

  // For each voxel position, compute distances to core points
  for (int i = 0; i < info.sizeX; i++)
  {
    for (int j = 0; j < info.sizeY; j++)
    {
      for (int k = 0; k < info.sizeZ; k++)
      { // for every scaled coordinate point
        pcl::PointXYZ test;
        test.x = i / info.scale - info.sizeX / (2 * info.scale);
        test.y = j / info.scale - info.sizeY / (2 * info.scale);
        test.z = k / info.scale -
                 info.sizeZ /
                   (2 * info.scale); // position of this voxel in world coordinates

        // Find distances from this voxel to its nearest two core points
        MazePoint mp;
        mp.setPoint(test);
        mp.setPoint2(-1);
        mp.setPoint1(-1);
        mp.setDist1(10000.0);
        mp.setDist2(100000.0); // set very large initial values
        for (int ii = 0; ii < numNodes; ii++)
        {
          double dist =
            std::sqrt((base[ii].x - test.x) * (base[ii].x - test.x) +
                      (base[ii].y - test.y) * (base[ii].y - test.y) +
                      (base[ii].z - test.z) * (base[ii].z - test.z));
          if (dist < mp.getDist1())
          {

            mp.setDist2(mp.getDist1());
            mp.setDist1(dist);

            mp.setPoint2(mp.getPoint1());
            mp.setPoint1(ii);
          }
          else if (dist < mp.getDist2())
          {
            mp.setDist2(dist);
            mp.setPoint2(ii);
          } // finding the distances to the nearest two cores
        }
        // Determine whether this voxel is a wall, road, or hole
        if (std::abs(mp.getDist2() - mp.getDist1()) < 1 / info.scale)
        { // the tested location is on one of the middle planes
          if ((mp.getPoint1() + mp.getPoint2()) >
                int((1 - connectivity) * numNodes) &&
              (mp.getPoint1() + mp.getPoint2()) <
                int((1 + connectivity) * numNodes))
          { // this is a holed wall (has an opening)
            double judge =
              std::sqrt((base[mp.getPoint1()].x - base[mp.getPoint2()].x) *
                          (base[mp.getPoint1()].x - base[mp.getPoint2()].x) +
                        (base[mp.getPoint1()].y - base[mp.getPoint2()].y) *
                          (base[mp.getPoint1()].y - base[mp.getPoint2()].y) +
                        (base[mp.getPoint1()].z - base[mp.getPoint2()].z) *
                          (base[mp.getPoint1()].z - base[mp.getPoint2()].z));
            if (mp.getDist1() + mp.getDist2() - judge >=
                roadRad / (info.scale * 3))
            {
              info.cloud->points.push_back(mp.getPoint());
            }
          }
          else
          {
            info.cloud->points.push_back(mp.getPoint());
          }
        }
      }
    }
  }

  info.cloud->width  = info.cloud->points.size();
  info.cloud->height = 1;
  RCLCPP_INFO(rclcpp::get_logger("Maze3DGen"), "the number of points before optimization is %d", info.cloud->width);
  info.cloud->points.resize(info.cloud->width * info.cloud->height);
  pcl2ros();
}
