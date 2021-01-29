#ifndef _CQUATERNION_H
#define _CQUATERNION_H

#include "vardef.h"

namespace msf {
namespace common {
  
    class Quaternion {
    public:
      /**
      * @brief Constructor which defers initialization quaternion
      */
      Quaternion()
      {
        quat_(0) = 1.0;
        quat_(1) = quat_(2) = quat_(3) = 0.0;
      }
      /**
       * @brief Destructor
       */
      virtual ~Quaternion() {}

       /**
       * @brief Get the quaternion.
       *
       * @return quaternion values
       */
      const Vector4d &GetQuaternion() const {return quat_;}
      
      /**
       * @brief Set the System quaternion.
       *
       * @return none
       */
      void SetQuaternion(const Vector4d &quat) {quat_ = quat;}
      /**
       * @brief Get System quaternion from attitude.
       *
       * @return none
       */
      void Att2Quat(const Vector3d att);
      /**
       * @brief Get Transform Matrix from System quaternion.
       *
       * @return none
       */
      void Cbn2Quat(const Matrix3d cbn);
      /**
       * @brief get angular and update system quaternion.
       *
       * @return none
       */
      void Update(const Vector3d ang);
      /**
       * @brief get transform matrix from attitude.
       *
       * @return none
       */
      void Att2Cbn(Matrix3d &cbn, const Vector3d att);
      /**
       * @brief Get Transform Matrix from quaternion.
       *
       * @return none
       */
      void Quat2Cbn(Matrix3d &cbn);
      /**
       * @brief get attitude from transform matrix.
       *
       * @return none
       */
      void Cbn2Att(Vector3d &att, const Matrix3d cbn);
      /**
       * @brief Get attitude from System quaternion.
       *
       * @return none
       */
      void Quat2Att(Vector3d &att);

    private:
      Vector4d quat_;

    private:
      FRIEND_TEST(quaternionTest, SetQuaternion);
      FRIEND_TEST(quaternionTest, Att2Quat);
      FRIEND_TEST(quaternionTest, Cbn2Quat);
      FRIEND_TEST(quaternionTest, Update);
      FRIEND_TEST(quaternionTest, Att2Cbn);
      FRIEND_TEST(quaternionTest, Quat2Cbn);
      FRIEND_TEST(quaternionTest, Cbn2Att);
      FRIEND_TEST(quaternionTest, Quat2Att);
    };

    inline void Quaternion::Att2Quat(const Vector3d att)
    {
      double cosR = cos(att(0) / 2.0);
      double sinR = sin(att(0) / 2.0);
      double cosP = cos(att(1) / 2.0);
      double sinP = sin(att(1) / 2.0);
      double cosH = cos(att(2) / 2.0);
      double sinH = sin(att(2) / 2.0);

      quat_(0) = cosH*cosP*cosR + sinH*sinP*sinR;
      quat_(1) = cosH*sinP*cosR + sinH*cosP*sinR;
      quat_(2) = cosH*cosP*sinR - sinH*sinP*cosR;
      quat_(3) = cosH*sinP*sinR - sinH*cosP*cosR;
    }

    inline void Quaternion::Cbn2Quat(const Matrix3d cbn)
    {
      double tr = cbn(0,0) + cbn(1,1) + cbn(2,2);

      if (tr > 0) {
        double S = sqrt(tr + 1.0) * 2.0;
        quat_(0) = 0.25 * S;
        quat_(1) = (cbn(2,1) - cbn(1,2)) / S;
        quat_(2) = (cbn(0,2) - cbn(2,0)) / S;
        quat_(3) = (cbn(1,0) - cbn(0,1)) / S;
      } else if ((cbn(0,0) > cbn(1,1)) && (cbn(0,0) > cbn(2,2))) {
        double S = sqrt(1.0 + cbn(0,0) - cbn(1,1) - cbn(2,2)) * 2.0;
        quat_(0) = (cbn(2,1) - cbn(1,2)) / S;
        quat_(1) = 0.25 * S;
        quat_(2) = (cbn(0,1) + cbn(1,0)) / S;
        quat_(3) = (cbn(0,2) + cbn(2,0)) / S;
      } else if (cbn(1,1) > cbn(2,2)) {
        double S = sqrt(1.0 - cbn(0,0) + cbn(1,1) - cbn(2,2)) * 2.0;
        quat_(0) = (cbn(0,2) - cbn(2,0)) / S;
        quat_(1) = (cbn(0,1) + cbn(1,0)) / S;
        quat_(2) = 0.25 * S;
          quat_(3) = (cbn(1,2) + cbn(2,1)) / S;
      } else {
        double S = sqrt(1.0 - cbn(0,0) - cbn(1,1) + cbn(2,2)) * 2.0;
        quat_(0) = -(cbn(1,0) - cbn(0,1)) / S;
        quat_(1) = -(cbn(0,2) + cbn(2,0)) / S;
        quat_(2) = -(cbn(1,2) + cbn(2,1)) / S;
        quat_(3) = -0.25 * S;
      }

      double normQ = quat_.norm();
      quat_(0) /= normQ;
      quat_(1) /= normQ;
      quat_(2) /= normQ;
      quat_(3) /= normQ;

    }

    inline void Quaternion::Update(const Vector3d ang)
    {
      Vector4d qd, qt;

      double n = ang(0) * ang(0) + ang(1) * ang(1) + ang(2) * ang(2);
      double nn = n * n;
      double s = 0.5 - n / 48.0 + nn / 3840.0 - n * nn / 645120.0 + nn * nn / 185794560.0;
      qt(0) = 1.0 - n / 8.0 + nn / 384.0 - n * nn / 46080.0 + nn * nn / 10321920.0;
      qt(1) = s * ang(0);
      qt(2) = s * ang(1);
      qt(3) = s * ang(2);

      qd(0) = quat_(0) * qt(0) - quat_(1) * qt(1) - quat_(2) * qt(2) - quat_(3) * qt(3);
      qd(1) = quat_(1) * qt(0) + quat_(0) * qt(1) - quat_(3) * qt(2) + quat_(2) * qt(3);
      qd(2) = quat_(2) * qt(0) + quat_(3) * qt(1) + quat_(0) * qt(2) - quat_(1) * qt(3);
      qd(3) = quat_(3) * qt(0) - quat_(2) * qt(1) + quat_(1) * qt(2) + quat_(0) * qt(3);

      double normQ = qd.norm();
      quat_(0) = qd(0) / normQ;
      quat_(1) = qd(1) / normQ;
      quat_(2) = qd(2) / normQ;
      quat_(3) = qd(3) / normQ;

    }

    inline void Quaternion::Att2Cbn(Matrix3d &cbn, const Vector3d att)
    {
      double cosR = cos(att(0));
      double sinR = sin(att(0));
      double cosP = cos(att(1));
      double sinP = sin(att(1));
      double cosH = cos(att(2));
      double sinH = sin(att(2));

      cbn(0,0) = cosR*cosH + sinR*sinP*sinH;
      cbn(0,1) = cosP*sinH;
      cbn(0,2) = sinR*cosH - cosR*sinP*sinH;
      cbn(1,0) = -cosR*sinH + sinR*sinP*cosH;
      cbn(1,1) = cosP*cosH;
      cbn(1,2) = -sinR*sinH - cosR*sinP*cosH;
      cbn(2,0) = -sinR*cosP;
      cbn(2,1) = sinP;
      cbn(2,2) = cosR*cosP;
      
    }

    inline void Quaternion::Quat2Cbn(Matrix3d &cbn)
    {
      double q00 = quat_(0) * quat_(0);
      double q01 = quat_(0) * quat_(1);
      double q02 = quat_(0) * quat_(2);
      double q03 = quat_(0) * quat_(3);
      double q11 = quat_(1) * quat_(1);
      double q12 = quat_(1) * quat_(2);
      double q13 = quat_(1) * quat_(3);
      double q22 = quat_(2) * quat_(2);
      double q23 = quat_(2) * quat_(3);
      double q33 = quat_(3) * quat_(3);

      cbn(0,0) = q00 + q11 - q22 - q33;
      cbn(0,1) = 2 * (q12 - q03);
      cbn(0,2) = 2 * (q13 + q02);

      cbn(1,0) = 2 * (q12 + q03);
      cbn(1,1) = q00 - q11 + q22 - q33;
      cbn(1,2) = 2 * (q23 - q01);

      cbn(2,0) = 2 * (q13 - q02);
      cbn(2,1) = 2 * (q23 + q01);
      cbn(2,2) = q00 - q11 - q22 + q33;

    }

    inline void Quaternion::Cbn2Att(Vector3d &att, const Matrix3d cbn)
    {

      att(0) = atan2(-cbn(2,0), cbn(2,2));  // Roll
      att(1) = asin(cbn(2,1));              // Pitch
      att(2) = atan2(cbn(0,1), cbn(1,1));   // Yaw

    }

    inline void Quaternion::Quat2Att(Vector3d &att)
    { 
      double q00 = quat_(0) * quat_(0);
      double q01 = quat_(0) * quat_(1);
      double q02 = quat_(0) * quat_(2);
      double q03 = quat_(0) * quat_(3);
      double q11 = quat_(1) * quat_(1);
      double q12 = quat_(1) * quat_(2);
      double q13 = quat_(1) * quat_(3);
      double q22 = quat_(2) * quat_(2);
      double q23 = quat_(2) * quat_(3);
      double q33 = quat_(3) * quat_(3);

      att(0) = atan2(-2.0 * (q13 - q02), q00 - q11 - q22 + q33);
      att(1) = asin(2.0 * (q01 + q23));
      att(2) = atan2(2.0 * (q12 - q03), q00 - q11 + q22 - q33);

    }

  }
}

#endif
