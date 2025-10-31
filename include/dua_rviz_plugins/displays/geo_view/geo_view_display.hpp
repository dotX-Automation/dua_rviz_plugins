#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <OgreMaterial.h>
#include <OgreManualObject.h>
#include <OgreSceneNode.h>
#include <OgreTexture.h>

#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkReply>
#include <QImage>

#include <rviz_common/logging.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

struct sqlite3;

namespace dua_rviz_plugins::displays::geo_view
{

using sensor_msgs::msg::NavSatFix;

class GeoViewDisplay : public rviz_common::MessageFilterDisplay<NavSatFix>
{
  Q_OBJECT

public:
  GeoViewDisplay();
  ~GeoViewDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;
  void processMessage(NavSatFix::ConstSharedPtr msg) override;

private:
  // Properties
  rviz_common::properties::EnumProperty * prop_source_type_;
  rviz_common::properties::StringProperty * prop_xyz_url_;
  rviz_common::properties::StringProperty * prop_mbtiles_path_;
  rviz_common::properties::IntProperty * prop_zoom_;
  rviz_common::properties::IntProperty * prop_radius_;
  rviz_common::properties::FloatProperty * prop_opacity_;
  rviz_common::properties::BoolProperty * prop_draw_behind_;

  // Scene root
  Ogre::SceneNode * scene_node_ {nullptr};

  // Per-tile visual
  struct TileVisual
  {
    Ogre::ManualObject * quad{nullptr};
    Ogre::MaterialPtr    material;
    Ogre::TexturePtr     texture;
  };
  std::unordered_map<std::string, TileVisual> visuals_; // key = "z_x_y"

  // Helpers/state
  std::unique_ptr<QNetworkAccessManager> net_;
  sqlite3 * mbtiles_db_ {nullptr};

  // Async state
  std::unordered_map<std::string, QNetworkReply *> pending_;  // key -> reply*
  std::unordered_set<std::string> have_texture_;             // keys with uploaded texture

  struct Tile { int z{0}, x{0}, y{0}; };
  Tile last_center_{};
  int  last_z_{-1};
  int  last_r_{-1};
  int  last_src_{-1};

  static Tile latLonToTile(double lat_deg, double lon_deg, int z);
  static double tileMetersAtLat(double lat_deg, int z); // 256px tile width in meters
  static std::string keyFor(const Tile & t);

  bool ensureMbtilesOpen(const std::string & path);
  bool loadTileXYZ(const Tile & t, QImage & out);
  bool loadTileMBTiles(const Tile & t, QImage & out);

  // Per-tile OGRE upload & quad build
  bool uploadToOgre(const QImage & img, TileVisual & vis, const std::string & key_hint);
  void createOrUpdateQuad(
    TileVisual & vis, double tile_meters, float opacity, bool draw_behind,
    float offset_x_m, float offset_y_m);

  // Visual lifecycle
  TileVisual & getOrCreateVisual(const std::string & key);
  void destroyVisual(const std::string & key);
  void destroyAllVisuals();

  // Cancel and clear all pending network replies
  void cancelAllPending();
};

}  // namespace dua_rviz_plugins::displays::geo_view
