#ifndef FLEET__MISSION_STORE_HPP_
#define FLEET__MISSION_STORE_HPP_

#include <cstdint>
#include <mutex>
#include <string>

struct sqlite3;

namespace fleet
{

/*
 * Durable audit trail for missions and incidents. Every other piece of
 * fleet state in this project lives only in process memory and is lost on
 * restart; for a security system that is close to disqualifying, since
 * "what did the fleet do and when" needs to survive a crash. This is
 * intentionally a single SQLite file rather than a database service -
 * the goal is a record that outlives the process, not a query API.
 */
class MissionStore
{
public:
  explicit MissionStore(const std::string & database_path);
  ~MissionStore();

  MissionStore(const MissionStore &) = delete;
  MissionStore & operator=(const MissionStore &) = delete;

  void record_mission_event(
    const std::string & mission_id,
    const std::string & robot_id,
    const std::string & mission_type,
    const std::string & command,
    const std::string & status,
    const std::string & detail);

  void record_incident(
    const std::string & incident_id,
    const std::string & robot_id,
    uint8_t severity,
    const std::string & category,
    const std::string & description);

  bool is_open() const;

private:
  void exec(const std::string & sql);

  sqlite3 * db_{nullptr};
  std::mutex mutex_;
};

}  // namespace fleet

#endif  // FLEET__MISSION_STORE_HPP_
