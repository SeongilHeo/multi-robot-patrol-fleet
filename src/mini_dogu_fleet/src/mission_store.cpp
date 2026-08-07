#include "mini_dogu_fleet/mission_store.hpp"

#include <sqlite3.h>

#include "rclcpp/rclcpp.hpp"

namespace mini_dogu_fleet
{

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("mission_store");
}

int bind_text(
  sqlite3_stmt * statement,
  int index,
  const std::string & value)
{
  return sqlite3_bind_text(
    statement,
    index,
    value.c_str(),
    static_cast<int>(value.size()),
    SQLITE_TRANSIENT);
}
}  // namespace

MissionStore::MissionStore(const std::string & database_path)
{
  if (sqlite3_open(database_path.c_str(), &db_) != SQLITE_OK) {
    RCLCPP_ERROR(
      logger(),
      "Failed to open mission store at '%s': %s",
      database_path.c_str(),
      db_ != nullptr ? sqlite3_errmsg(db_) : "unknown error");

    if (db_ != nullptr) {
      sqlite3_close(db_);
      db_ = nullptr;
    }

    return;
  }

  exec(
    "CREATE TABLE IF NOT EXISTS mission_events ("
    "id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "mission_id TEXT,"
    "robot_id TEXT,"
    "mission_type TEXT,"
    "command TEXT,"
    "status TEXT,"
    "detail TEXT,"
    "created_at TEXT DEFAULT CURRENT_TIMESTAMP)");

  exec(
    "CREATE TABLE IF NOT EXISTS incidents ("
    "id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "incident_id TEXT,"
    "robot_id TEXT,"
    "severity INTEGER,"
    "category TEXT,"
    "description TEXT,"
    "created_at TEXT DEFAULT CURRENT_TIMESTAMP)");

  RCLCPP_INFO(
    logger(),
    "Mission store opened at '%s'",
    database_path.c_str());
}

MissionStore::~MissionStore()
{
  if (db_ != nullptr) {
    sqlite3_close(db_);
  }
}

bool MissionStore::is_open() const
{
  return db_ != nullptr;
}

void MissionStore::exec(const std::string & sql)
{
  if (db_ == nullptr) {
    return;
  }

  char * error_message = nullptr;

  if (
    sqlite3_exec(
      db_,
      sql.c_str(),
      nullptr,
      nullptr,
      &error_message) != SQLITE_OK)
  {
    RCLCPP_ERROR(
      logger(),
      "Mission store SQL error: %s",
      error_message != nullptr ? error_message : "unknown error");

    sqlite3_free(error_message);
  }
}

void MissionStore::record_mission_event(
  const std::string & mission_id,
  const std::string & robot_id,
  const std::string & mission_type,
  const std::string & command,
  const std::string & status,
  const std::string & detail)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (db_ == nullptr) {
    return;
  }

  static const char * const kSql =
    "INSERT INTO mission_events "
    "(mission_id, robot_id, mission_type, command, status, detail) "
    "VALUES (?, ?, ?, ?, ?, ?)";

  sqlite3_stmt * statement = nullptr;

  if (sqlite3_prepare_v2(db_, kSql, -1, &statement, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(
      logger(),
      "Failed to prepare mission_events insert: %s",
      sqlite3_errmsg(db_));
    return;
  }

  bind_text(statement, 1, mission_id);
  bind_text(statement, 2, robot_id);
  bind_text(statement, 3, mission_type);
  bind_text(statement, 4, command);
  bind_text(statement, 5, status);
  bind_text(statement, 6, detail);

  if (sqlite3_step(statement) != SQLITE_DONE) {
    RCLCPP_ERROR(
      logger(),
      "Failed to insert mission event: %s",
      sqlite3_errmsg(db_));
  }

  sqlite3_finalize(statement);
}

void MissionStore::record_incident(
  const std::string & incident_id,
  const std::string & robot_id,
  uint8_t severity,
  const std::string & category,
  const std::string & description)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (db_ == nullptr) {
    return;
  }

  static const char * const kSql =
    "INSERT INTO incidents "
    "(incident_id, robot_id, severity, category, description) "
    "VALUES (?, ?, ?, ?, ?)";

  sqlite3_stmt * statement = nullptr;

  if (sqlite3_prepare_v2(db_, kSql, -1, &statement, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(
      logger(),
      "Failed to prepare incidents insert: %s",
      sqlite3_errmsg(db_));
    return;
  }

  bind_text(statement, 1, incident_id);
  bind_text(statement, 2, robot_id);
  sqlite3_bind_int(statement, 3, static_cast<int>(severity));
  bind_text(statement, 4, category);
  bind_text(statement, 5, description);

  if (sqlite3_step(statement) != SQLITE_DONE) {
    RCLCPP_ERROR(
      logger(),
      "Failed to insert incident: %s",
      sqlite3_errmsg(db_));
  }

  sqlite3_finalize(statement);
}

}  // namespace mini_dogu_fleet
