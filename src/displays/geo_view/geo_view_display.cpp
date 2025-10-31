#include <cmath>
#include <sstream>
#include <unordered_set>

#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkRequest>
#include <QtNetwork/QNetworkReply>
#include <QtNetwork/QNetworkDiskCache>
#include <QEventLoop>
#include <QUrl>
#include <QStandardPaths>

#include <sqlite3.h>

#include <OgreHardwareBufferManager.h>
#include <OgreImage.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include <rviz_common/uniform_string_stream.hpp>
#include <dua_rviz_plugins/displays/geo_view/geo_view_display.hpp>

namespace dua_rviz_plugins::displays::geo_view
{

// Helper for QObject unique_ptr guards
struct QObjectDeleter { void operator()(QObject * o) const {if (o) {o->deleteLater();}} };

// ------ small helpers ------
GeoViewDisplay::Tile GeoViewDisplay::latLonToTile(double lat_deg, double lon_deg, int z)
{
  const double lat_rad = lat_deg * M_PI / 180.0;
  const double n = std::pow(2.0, z);
  const int x = static_cast<int>(std::floor((lon_deg + 180.0) / 360.0 * n));
  const int y = static_cast<int>(std::floor((1.0 -
    std::log(std::tan(lat_rad) + 1.0 / std::cos(lat_rad)) / M_PI) / 2.0 * n));
  return {z, x, y};
}

double GeoViewDisplay::tileMetersAtLat(double lat_deg, int z)
{
  const double earth_circum = 40075016.68557849;
  const double m_per_px = std::cos(lat_deg * M_PI / 180.0) * earth_circum / (256.0 * std::pow(2.0,
      z));
  return m_per_px * 256.0;
}

std::string GeoViewDisplay::keyFor(const Tile & t)
{
  std::ostringstream os;
  os << t.z << "_" << t.x << "_" << t.y;
  return os.str();
}

// ------ ctor/dtor ------
GeoViewDisplay::GeoViewDisplay()
{
  prop_source_type_ = new rviz_common::properties::EnumProperty(
    "Source Type", "XYZ", "Select between XYZ (online) or MBTiles (offline).", this);
  prop_source_type_->addOption("XYZ", 0);
  prop_source_type_->addOption("MBTiles", 1);

  prop_xyz_url_ = new rviz_common::properties::StringProperty(
    "XYZ URL",
      "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
      "URL template for the tile provider.", this);

  prop_mbtiles_path_ = new rviz_common::properties::StringProperty(
    "MBTiles Path", "", "Path to the MBTiles file to use as a tile source.", this);

  prop_zoom_ = new rviz_common::properties::IntProperty("Zoom Level", 17, "Zoom level.", this);
  prop_zoom_->setMin(1); prop_zoom_->setMax(20);

  prop_radius_ = new rviz_common::properties::IntProperty(
    "Radius", 1, "Number of tiles around the GPS fix to display.", this);
  prop_radius_->setMin(0); prop_radius_->setMax(5);

  prop_opacity_ = new rviz_common::properties::FloatProperty(
    "Opacity", 1.0f, "Opacity of the tiles.", this);
  prop_opacity_->setMin(0.0); prop_opacity_->setMax(1.0);

  prop_draw_behind_ = new rviz_common::properties::BoolProperty(
    "Draw Behind", true, "Disable depth write so the map sits behind objects.", this);
}

GeoViewDisplay::~GeoViewDisplay()
{
  cancelAllPending();
  destroyAllVisuals();
  if (scene_node_) {
    scene_node_->detachAllObjects();
    scene_node_->getParentSceneNode()->removeAndDestroyChild(scene_node_->getName());
    scene_node_ = nullptr;
  }
  if (mbtiles_db_) {
    sqlite3_close(mbtiles_db_);
    mbtiles_db_ = nullptr;
  }
}

// ------ rviz lifecycle ------
void GeoViewDisplay::onInitialize()
{
  rviz_common::MessageFilterDisplay<NavSatFix>::onInitialize();

  net_ = std::make_unique<QNetworkAccessManager>(this);

  // Disk cache to keep things snappy
  auto *cache = new QNetworkDiskCache(this);
  auto dir = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
  cache->setCacheDirectory(dir + "/geo_view_tiles");
  cache->setMaximumCacheSize(256 * 1024 * 1024); // 256 MB
  net_->setCache(cache);

  // Scene node for our tiles
  Ogre::SceneManager * sm = context_->getSceneManager();
  Ogre::SceneNode * root = sm->getRootSceneNode();
  rviz_common::UniformStringStream ss; ss << "GeoViewNode" << this;
  scene_node_ = root->createChildSceneNode(ss.str());
}

void GeoViewDisplay::reset()
{
  rviz_common::MessageFilterDisplay<NavSatFix>::reset();
  cancelAllPending();
  have_texture_.clear();
  destroyAllVisuals();
  last_z_ = last_r_ = last_src_ = -1;
  last_center_ = {};
}

// ------ tile IO ------
bool GeoViewDisplay::ensureMbtilesOpen(const std::string & path)
{
  if (!mbtiles_db_) {
    if (path.empty()) {return false;}
    if (sqlite3_open(path.c_str(), &mbtiles_db_) != SQLITE_OK) {
      RVIZ_COMMON_LOG_ERROR_STREAM("MBTiles open failed: " << sqlite3_errmsg(mbtiles_db_));
      sqlite3_close(mbtiles_db_);
      mbtiles_db_ = nullptr;
      return false;
    }
  }
  return true;
}

// Fast path: try to return from network disk cache synchronously.
// If not in cache (or network required), return false (async path will take over).
bool GeoViewDisplay::loadTileXYZ(const Tile & t, QImage & out)
{
  QString url_tmpl = QString::fromStdString(prop_xyz_url_->getStdString());
  url_tmpl.replace("{z}", QString::number(t.z));
  url_tmpl.replace("{x}", QString::number(t.x));
  url_tmpl.replace("{y}", QString::number(t.y));

  QNetworkRequest req{QUrl(url_tmpl)};
  req.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::PreferCache);

  // synchronous only to hit cache quickly
  QNetworkReply * r = net_->get(req);
  QEventLoop loop;
  QObject::connect(r, &QNetworkReply::finished, &loop, &QEventLoop::quit);
  loop.exec();
  if (r->error() != QNetworkReply::NoError) {r->deleteLater(); return false;}

  const bool from_cache = r->attribute(QNetworkRequest::SourceIsFromCacheAttribute).toBool();
  QByteArray data = r->readAll();
  r->deleteLater();
  if (!from_cache) {return false;}
  if (!out.loadFromData(data)) {return false;}
  out = out.convertToFormat(QImage::Format_RGBA8888);
  return true;
}

bool GeoViewDisplay::loadTileMBTiles(const Tile & t, QImage & out)
{
  const std::string path = prop_mbtiles_path_->getStdString();
  if (!ensureMbtilesOpen(path)) {return false;}

  // MBTiles uses TMS y => flip y: y_tms = (2^z - 1 - y_xyz)
  const int n = static_cast<int>(std::pow(2.0, t.z));
  const int y_tms = (n - 1 - t.y);

  const char * sql =
    "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=? LIMIT 1;";
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(mbtiles_db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RVIZ_COMMON_LOG_ERROR_STREAM("MBTiles prepare failed: " << sqlite3_errmsg(mbtiles_db_));
    return false;
  }
  sqlite3_bind_int(stmt, 1, t.z);
  sqlite3_bind_int(stmt, 2, t.x);
  sqlite3_bind_int(stmt, 3, y_tms);

  bool ok = false;
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    const void * blob = sqlite3_column_blob(stmt, 0);
    const int bytes = sqlite3_column_bytes(stmt, 0);
    QByteArray data(reinterpret_cast<const char *>(blob), bytes);
    ok = out.loadFromData(data);
    if (ok) {out = out.convertToFormat(QImage::Format_RGBA8888);}
  } else {
    RVIZ_COMMON_LOG_WARNING_STREAM(
      "Tile not found in MBTiles: z=" << t.z << " x=" << t.x << " y_tms=" << y_tms);
  }
  sqlite3_finalize(stmt);
  return ok;
}

// ------ per-tile visuals ------
GeoViewDisplay::TileVisual & GeoViewDisplay::getOrCreateVisual(const std::string & key)
{
  auto it = visuals_.find(key);
  if (it != visuals_.end()) {return it->second;}

  // Create quad and material for this tile
  TileVisual vis;
  // material
  rviz_common::UniformStringStream ms; ms << "GeoViewMaterial_" << key;
  vis.material = Ogre::MaterialManager::getSingleton().create(
    ms.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  vis.material->setReceiveShadows(false);
  vis.material->getTechnique(0)->setLightingEnabled(false);
  vis.material->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
  vis.material->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);
  vis.material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  vis.material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

  // quad
  rviz_common::UniformStringStream qs; qs << "GeoViewQuad_" << key;
  vis.quad = context_->getSceneManager()->createManualObject(qs.str());
  vis.quad->setDynamic(true);
  scene_node_->attachObject(vis.quad);

  auto [ins_it, _] = visuals_.emplace(key, std::move(vis));
  return ins_it->second;
}

void GeoViewDisplay::destroyVisual(const std::string & key)
{
  auto it = visuals_.find(key);
  if (it == visuals_.end()) {return;}
  TileVisual & vis = it->second;
  if (vis.quad) {
    scene_node_->detachObject(vis.quad);
    context_->getSceneManager()->destroyManualObject(vis.quad);
    vis.quad = nullptr;
  }
  if (vis.texture) {
    Ogre::TextureManager::getSingleton().remove(vis.texture->getName());
    vis.texture.reset();
  }
  if (vis.material) {
    Ogre::MaterialManager::getSingleton().remove(vis.material->getName());
    vis.material.reset();
  }
  visuals_.erase(it);
}

void GeoViewDisplay::destroyAllVisuals()
{
  std::vector<std::string> keys;
  keys.reserve(visuals_.size());
  for (auto & kv : visuals_) {
    keys.push_back(kv.first);
  }
  for (auto & k : keys) {
    destroyVisual(k);
  }
}

void GeoViewDisplay::cancelAllPending()
{
  for (auto & kv : pending_) {
    if (kv.second) {kv.second->abort(); kv.second->deleteLater();}
  }
  pending_.clear();
}

// ------ ogre upload & draw ------
bool GeoViewDisplay::uploadToOgre(
  const QImage & img, TileVisual & vis,
  const std::string & key_hint)
{
  Ogre::Image ogre_img;
  QImage copy = img;
  const int w = copy.width(), h = copy.height();
  ogre_img.loadDynamicImage(copy.bits(), w, h, 1, Ogre::PF_A8B8G8R8, false /*autoDelete*/);

  // Remove old texture (if any), then load
  if (vis.texture) {
    Ogre::TextureManager::getSingleton().remove(vis.texture->getName());
    vis.texture.reset();
  }

  std::string tex_name = "GeoViewTexture_" + key_hint;
  vis.texture = Ogre::TextureManager::getSingleton().loadImage(
    tex_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, ogre_img);

  // Bind into material
  Ogre::Pass * pass = vis.material->getTechnique(0)->getPass(0);
  pass->removeAllTextureUnitStates();
  Ogre::TextureUnitState * tus = pass->createTextureUnitState(vis.texture->getName());
  tus->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
  return true;
}

void GeoViewDisplay::createOrUpdateQuad(
  TileVisual & vis, double tile_meters, float opacity,
  bool draw_behind, float offset_x_m, float offset_y_m)
{
  const float half = static_cast<float>(tile_meters * 0.5);

  // material states per tile
  Ogre::Pass * pass = vis.material->getTechnique(0)->getPass(0);
  pass->setDepthWriteEnabled(!draw_behind ? true : false);
  pass->setAlphaRejectSettings(Ogre::CMPF_ALWAYS_PASS, 0);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDiffuse(1, 1, 1, opacity);
  pass->setAmbient(1, 1, 1);

  vis.quad->clear();
  vis.quad->begin(vis.material->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

  // Quad vertices in XY plane at z=0, translated by offsets
  const float ox = offset_x_m;
  const float oy = offset_y_m;

  vis.quad->position(ox - half, oy - half, 0.0f); vis.quad->textureCoord(0.0f, 1.0f);
  vis.quad->position(ox + half, oy - half, 0.0f); vis.quad->textureCoord(1.0f, 1.0f);
  vis.quad->position(ox + half, oy + half, 0.0f); vis.quad->textureCoord(1.0f, 0.0f);

  vis.quad->position(ox - half, oy - half, 0.0f); vis.quad->textureCoord(0.0f, 1.0f);
  vis.quad->position(ox + half, oy + half, 0.0f); vis.quad->textureCoord(1.0f, 0.0f);
  vis.quad->position(ox - half, oy + half, 0.0f); vis.quad->textureCoord(0.0f, 0.0f);

  vis.quad->end();
}

// ------ main message handler ------
void GeoViewDisplay::processMessage(NavSatFix::ConstSharedPtr msg)
{
  if (!isEnabled()) {return;}

  const int z = prop_zoom_->getInt();
  const int r = prop_radius_->getInt();
  const int src = prop_source_type_->getOptionInt();
  const float opacity = prop_opacity_->getFloat();
  const bool draw_behind = prop_draw_behind_->getBool();

  const Tile center = latLonToTile(msg->latitude, msg->longitude, z);
  const double tile_m = tileMetersAtLat(msg->latitude, z);
  const int n = static_cast<int>(std::pow(2.0, z));

  const bool tiling_changed = (
    z != last_z_ || r != last_r_ || src != last_src_ ||
    center.x != last_center_.x || center.y != last_center_.y || center.z != last_center_.z);

  // If nothing in the tiling changed, just reposition/update quads (cheap)
  if (!tiling_changed) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        Tile t{z, center.x + dx, center.y + dy};
        if (t.x < 0) {t.x = (t.x % n + n) % n;} else if (t.x >= n) {t.x = t.x % n;}
        if (t.y < 0) {t.y = 0;} else if (t.y >= n) {t.y = n - 1;}
        const std::string key = keyFor(t);
        auto it = visuals_.find(key);
        if (it == visuals_.end()) {continue;}
        const float off_x = static_cast<float>(dx * tile_m);
        const float off_y = static_cast<float>(-dy * tile_m);
        createOrUpdateQuad(it->second, tile_m, opacity, draw_behind, off_x, off_y);
      }
    }
    return;
  }

  // record state
  last_center_ = center; last_z_ = z; last_r_ = r; last_src_ = src;

  std::unordered_set<std::string> alive;
  alive.reserve((2 * r + 1) * (2 * r + 1));

  // async launcher for XYZ
  auto start_async_xyz = [&](const Tile & t, const std::string & key){
      if (pending_.count(key)) {return;} // already in flight
      QString url = QString::fromStdString(prop_xyz_url_->getStdString());
      url.replace("{z}", QString::number(t.z));
      url.replace("{x}", QString::number(t.x));
      url.replace("{y}", QString::number(t.y));

      QNetworkRequest req{QUrl(url)};
      req.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::PreferCache);
      auto *reply = net_->get(req);
      pending_[key] = reply;

      QObject::connect(reply, &QNetworkReply::finished, this, [this, reply, key]() {
          std::unique_ptr<QNetworkReply, QObjectDeleter> guard(reply);
          pending_.erase(key);
          if (reply->error() != QNetworkReply::NoError) {return;}

          QByteArray data = reply->readAll();
          QImage img;
          if (!img.loadFromData(data)) {return;}
          img = img.convertToFormat(QImage::Format_RGBA8888);

          auto it = visuals_.find(key);
          if (it == visuals_.end()) {return;}
          uploadToOgre(img, it->second, key);
          have_texture_.insert(key);
      // geometry will be positioned on the next processMessage()
    });
    };

  // Build a grid of tiles (2r+1)^2
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      Tile t{z, center.x + dx, center.y + dy};

      // Wrap X (world repeats east-west)
      if (t.x < 0) {t.x = (t.x % n + n) % n;}
      if (t.x >= n) {t.x = t.x % n;}

      // Clamp Y (no wrap north-south)
      if (t.y < 0) {t.y = 0;}
      if (t.y >= n) {t.y = n - 1;}

      const std::string key = keyFor(t);
      alive.insert(key);

      TileVisual & vis = getOrCreateVisual(key);

      // If we already have a texture, just position/update quad
      if (have_texture_.count(key)) {
        const float off_x = static_cast<float>(dx * tile_m);
        const float off_y = static_cast<float>(-dy * tile_m);
        createOrUpdateQuad(vis, tile_m, opacity, draw_behind, off_x, off_y);
        continue;
      }

      // Try fast path: cache for XYZ, local DB for MBTiles
      QImage img;
      bool ok = false;
      if (src == 0) {
        ok = loadTileXYZ(t, img);  // returns true only if served from disk cache
        if (!ok) {
          // queue async fetch; we'll upload when it completes
          start_async_xyz(t, key);
          continue;
        }
      } else {
        ok = loadTileMBTiles(t, img); // local DB: synchronous but quick
        if (!ok) {
          RVIZ_COMMON_LOG_WARNING_STREAM("MBTiles tile missing for key=" << key);
          continue;
        }
      }

      // upload and place
      uploadToOgre(img, vis, key);
      have_texture_.insert(key);

      const float off_x = static_cast<float>(dx * tile_m);
      const float off_y = static_cast<float>(-dy * tile_m);
      createOrUpdateQuad(vis, tile_m, opacity, draw_behind, off_x, off_y);
    }
  }

  // Cleanup tiles outside current radius
  std::vector<std::string> to_remove;
  to_remove.reserve(visuals_.size());
  for (auto & kv : visuals_) {
    if (!alive.count(kv.first)) {to_remove.push_back(kv.first);}
  }
  for (auto & k : to_remove) {
    auto it = pending_.find(k);
    if (it != pending_.end()) {it->second->abort(); it->second->deleteLater(); pending_.erase(it);}
    have_texture_.erase(k);
    destroyVisual(k);
  }

  RVIZ_COMMON_LOG_INFO_STREAM("Rendered tiles around z/x/y "
    << center.z << "/" << center.x << "/" << center.y
    << " radius=" << r << " ~tile_m=" << tile_m);
}

} // namespace dua_rviz_plugins::displays::geo_view

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  dua_rviz_plugins::displays::geo_view::GeoViewDisplay,
  rviz_common::Display)
