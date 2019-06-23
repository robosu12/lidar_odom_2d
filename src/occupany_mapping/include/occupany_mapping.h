#ifndef OCCUPANY_MAPPING_H
#define OCCUPANY_MAPPING_H

#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "readfile.h"




/**************************************************************************
 * Map manipulation macros
 **************************************************************************/

// Convert from map index to world coords  地图坐标转换到世界坐标
#define MAP_MXtoWX(map, i) (map->origin_x + ((i) - map->offset_x) * map->resolution)
#define MAP_MYtoWY(map, j) (map->origin_y + ((j) - map->offset_y) * map->resolution)

// Convert from world coords to map coords 世界坐标转换到地图坐标
#define MAP_WXtoMX(map, x) (floor((x - map->origin_x) / map->resolution + 0.5) + map->offset_x)
#define MAP_WYtoMY(map, y) (floor((y - map->origin_y) / map->resolution + 0.5) + map->offset_y)

// Test to see if the given map coords lie within the absolute map bounds. 判断是否出界
#define MAP_VALID(map, i, j) ((i >= 0) && (i < map->width) && (j >= 0) && (j < map->height))

// Compute the cell index for the given map coords. 把地图坐标转化为Index
#define MAP_INDEX(map, i, j) ((i) + (j) * map->width

typedef struct gridindex_
{
    int x;
    int y;

    void SetIndex(int x_,int y_)
    {
        x  = x_;
        y  = y_;
    }
}MapCoords;

// Description for a single map cell.
/**
 *单独的地图cell类型
 */
typedef struct
{
  // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  char occ_state;
  // Distance to the nearest occupied cell
//   float occ_dist;
  // logodd_val occupied cell
  double logodd_val;

  double score;

  // Wifi levels
  //int wifi_levels[MAP_WIFI_MAX_LEVELS];
} map_cell_t;


// Description for a map
/**
 *地图的数据结构
 */
class map_t
{
    public:
    // Map origin; the map is a viewport onto a conceptual larger map.
    double origin_x, origin_y;
    // Map scale (m/cell) 地图的分辨率
    double resolution;
    // Map dimensions (number of cells) X Y方向的栅格束
    int height, width;

    int offset_x,offset_y;

    double log_occ,log_free;
    double log_max,log_min;

    // The map data, stored as a grid
    map_cell_t *cells;

    // Max distance at which we care about obstacles, for constructing
    // likelihood field
    double max_occ_dist; //在似然场模型中，障碍物影响的最大距离

    double min_score;

    double likelihood_sigma; //似然场的标准差

    double getCellProbability(int index) 
    {
        double log_val = cells[index].logodd_val ;
        double odds = exp(log_val);
        return odds / (odds + 1.0);
    }

};

typedef struct map_params
{
    double log_occ,log_free;
    double log_max,log_min;
    double resolution;
    double origin_x,origin_y;
    int height,width;
    int offset_x,offset_y;
    map_cell_t cells[];
}MapParams;

// float* pMap;
// MapParams mapParams;

// map_t mapParams;

// map_t *map;

extern map_t *map;

#endif

