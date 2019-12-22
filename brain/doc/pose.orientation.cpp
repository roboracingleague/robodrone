// Catch pose.orientation on a real drone and convert it to euler angles
// Done for better orientation topic measures understanding
#include <iostream>
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

int printRes(std::string label, double x, double y, double z, double w) {
  Quaternion Mine;
  EulerAngles res;
  
  Mine.x=x;
  Mine.y=y;
  Mine.z=z;
  Mine.w=w;  
  res = ToEulerAngles(Mine);
  
  std::cout << label << "\n";
  std::cout << res.roll << " " << res.pitch << " " << res.yaw << "\n";
  return 0;
}

int main()
{
  printRes("position depart", 0.0193586848275, -0.0225472496631, -0.712623694997, -0.700916782206);
  printRes("demi-tour", 0.00620759133894, 0.0237090477926, -0.679388669187, 0.733369225332);
  printRes("pitch avant 180", -0.724723893861, 0.688046981888, 0.0319568877255, -0.0185870407333);
  printRes("pitch avant 90", -0.513452031046, 0.515543382035, 0.491898704005, 0.478139927212);
  printRes("pitch arriere 90", 0.503456999764, -0.452561060827, 0.523958903863, 0.516901015661);
  printRes("roll droit 90", 0.429190504253, 0.556722653162, 0.524600224469, 0.480260470162);
  printRes("roll gauche 90", -0.475554906234, -0.52183348933, 0.506542462542, 0.494926506601);
  printRes("turn droit 90", -0.00753887330614, 0.0295857778096, 0.0556722779602, 0.99798224466);
  printRes("turn droit 90 pitch avant 90", -0.0639759868336, 0.712728306768, 0.0181078244803, 0.698281884385);
  printRes("turn droit 90 pitch arriere 90", 0.0519466862127, -0.686381468826, 0.0351697959881, 0.724531025931);
  printRes("turn droit 90 roll droit", 0.696424620285, 0.0630987634021, 0.0086601447712, 0.714798180645);
  printRes("turn droit 90 roll gauche", -0.717732760339, -0.0364590470107, 0.0398813078895, 0.694219012482);
  printRes("turn droit 90 pitch avant 180", -0.0663547760369, 0.997385403138, -0.0189407405433, -0.021466167523);
  printRes("turn droit 90 roll droit 180", 0.997110437099, 0.0715508924025, 0.0236159658191, 0.0096804422654);
  printRes("demi-tour pitch avant 90", 0.466744252433, 0.55175912418, -0.493781146237, 0.483623658281);
  printRes("demi-tour pitch arriere 90", -0.455989922956, -0.506178795826, -0.500829995916, 0.533877916718);
  printRes("demi-tour roll droit 90", 0.522995541164, -0.438124311145, -0.473010485185, 0.5574800452);
  printRes("demi-tour roll gauche 90", -0.512458586692, 0.445311349804, -0.479984568989, 0.55560674159);
  printRes("turn gauche 90 ",0.0113920343445,-0.000159073401537,-0.999408578307,0.0324455712144);
  printRes("turn gauche 90 pitch avant 90",0.719323403476,0.042902604725,-0.691293810461,0.0533486168347);
  printRes("turn gauche 90 pitch arriere 90",-0.672859895814,-0.0725704246679,-0.736084931174,0.0131240228641);
  printRes("turn gauche 90 roll droit",0.0577137237881,-0.726329769132,-0.683433928976,0.0450819339678);
  printRes("turn gauche 90 roll gauche",-0.0397701877234,0.696369281473,-0.71658088697,0.000156701982763);
  printRes("turn gauche 90 pitch avant 180",0.994888033777,0.0906608635732,0.0409049415701,0.0174764673237);
  printRes("turn gauche 90 roll droit 180",0.0752922681448,-0.996738280426,0.0176270357876,0.0230909002157);
}
