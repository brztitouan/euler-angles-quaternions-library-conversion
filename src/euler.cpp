#include <iostream>
#include "../include/euler/io.h"
#include "../include/euler/rotations.h"

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

bool radians = false; // Get radians / degrees - Default false
bool intrinsic = true; // Default true
bool extrinsic = false; // Default false
bool active = true; // Default true
bool passive = false; // Default false
const std::string sequence = "zyx"; //The rotation sequence (possible values: xyz, xzy, yxz, yzx, zxy, zyx, xyx, xzx, yxy, yzy, zxz, zyz; default: zyx)

float zyx[3];
double yaw, pitch, roll;

struct Quaternion //typedef
{
    double w, x, y, z;
};

using namespace std;

void GetEulerAngles(Quaternion q, double& yaw, double& pitch, double& roll);

void GetEulerAngles(Quaternion q, double& yaw, double& pitch, double& roll)
{
    const double w2 = q.w*q.w;
    const double x2 = q.x*q.x;
    const double y2 = q.y*q.y;
    const double z2 = q.z*q.z;
    const double unitLength = w2 + x2 + y2 + z2;    // Normalised == 1, otherwise correction divisor.
    const double abcd = q.w*q.x + q.y*q.z;
    const double eps = 1e-7;    // TODO: pick from your math lib instead of hardcoding.
    const double pi = 3.14159265358979323846;   // TODO: pick from your math lib instead of hardcoding.
    if (abcd > (0.5-eps)*unitLength)
    {
        yaw = 2 * atan2(q.y, q.w);
        pitch = pi;
        roll = 0;
    }
    else if (abcd < (-0.5+eps)*unitLength)
    {
        yaw = -2 * ::atan2(q.y, q.w);
        pitch = -pi;
        roll = 0;
    }
    else
    {
        const double adbc = q.w*q.z - q.x*q.y;
        const double acbd = q.w*q.y - q.x*q.z;
        yaw = ::atan2(2*adbc, 1 - 2*(z2+x2));
        pitch = ::asin(2*abcd/unitLength);
        roll = ::atan2(2*acbd, 1 - 2*(y2+x2));
    }
}

int main()
{

  Quaternion quat;

  zyx[0] = 45;
  zyx[1] = 2;
  zyx[2] = 2;

  // Get intrinsic / extrinsic
  if (intrinsic && extrinsic)
  {
    std::cerr << "Error: Cannot use both intrinsic and extrinsic elemental "
                 "rotations.\n\n"
              << std::endl;
    return -1;
  }

  euler::Order order =
      extrinsic ? euler::Order::EXTRINSIC : euler::Order::INTRINSIC;

  // Get active // passive
  if (active && passive)
  {
    std::cerr << "Error: Cannot specify both an active and passive rotation.\n\n"
              << std::endl;
    return -1;
  }
  euler::Direction direction =
      passive ? euler::Direction::PASSIVE : euler::Direction::ACTIVE;

  // Get sequence
  if (!euler::isSequenceValid(sequence))
  {
    std::cerr << "Error: Invalid sequence.\n\n" << std::endl;
    return -1;
  }

  // Get angles
  euler::Angles angles;
  try
  {
    angles = {zyx[0],zyx[1],zyx[2]};
  }
  catch (const std::invalid_argument&)
  {
    std::cerr << "Error: Invalid angles, must be three real numbers.\n\n"
              << std::endl;
    return -1;
  }

  // Convert / verify angles
  if (!radians)
  {
    std::for_each(angles.begin(), angles.end(),
                  [](double& a) { a *= M_PI / 180; });
  }

  // Create rotation matrix and quaternion from arguments
  const euler::RotationMatrix R =
      euler::getRotationMatrix(sequence, angles, {order, direction});
  const euler::Quaternion q = euler::getQuaternion(R);

  // Display results
  std::cout << std::endl;
  euler::prettyPrint(R);
  std::cout << std::endl;
  euler::prettyPrint(q);

  quat.w = q.w();
  quat.x = q.x();
  quat.y = q.y();
  quat.z = q.z();

  printf("\nEuler angles value before quaternions conversion:\n");
  printf("\nIn Degrees >\tyaw: %f\n\t\tpitch: %f\n\t\troll: %f\n", zyx[0], zyx[1], zyx[2]);
  printf("\nIn Radians >\tyaw: %f\n\t\tpitch: %f\n\t\troll: %f\n", degreesToRadians(zyx[0]), degreesToRadians(zyx[1]), degreesToRadians(zyx[2]));

  GetEulerAngles(quat, yaw, pitch, roll);
  printf("\nEuler angles value after quaternions conversion:\n");
  printf("\nIn Radians >\tyaw: %f\n\t\tpitch: %f\n\t\troll: %f\n", yaw, pitch, roll);
  printf("\nIn Degrees >\tyaw: %f\n\t\tpitch: %f\n\t\troll: %f\n", radiansToDegrees(yaw), radiansToDegrees(pitch), radiansToDegrees(roll));
  return 0;
}
