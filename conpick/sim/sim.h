#if !defined(sim_sim_h)
#define sim_sim_h

#include "Framework/Test.h"

struct Arm
{
    b2RevoluteJoint* mpSh;
    b2RevoluteJoint* mpEl;
    b2RevoluteJoint* mpWr;
    b2PrismaticJoint* mpGr0;
    b2PrismaticJoint* mpGr1;
};

void SetUpWorld(b2World* pWorld);
Arm CreateArm(b2World* pWorld);

class Sim : public Test
{
public:
    Sim()
    {
        SetUpWorld(m_world);
        mArm = CreateArm(m_world);
    }

    void Keyboard(int key)
    {
        switch (key)
        {
            case GLFW_KEY_L:
                mArm.mpSh->EnableLimit(!mArm.mpSh->IsLimitEnabled());
                break;

            case GLFW_KEY_M:
                mArm.mpSh->EnableMotor(!mArm.mpSh->IsMotorEnabled());
                break;
        }
    }

    void Step(Settings* settings)
    {
        Test::Step(settings);
        g_debugDraw.DrawString(5, m_textLine, "Keys: (l) limits, (m) motor");
        m_textLine += DRAW_STRING_NEW_LINE;

        //if (m_stepCount == 360)
        //{
        //  m_ball->SetTransform(b2Vec2(0.0f, 0.5f), 0.0f);
        //}

        //float32 torque1 = m_joint1->GetMotorTorque();
        //g_debugDraw.DrawString(5, m_textLine, "Motor Torque = %4.0f, %4.0f : Motor Force = %4.0f", (float) torque1, (float) torque2, (float) force3);
        //m_textLine += DRAW_STRING_NEW_LINE;
    }

    static Test* Create()
    {
        return new Sim();
    }

    Arm mArm;
};

#endif /* sim_sim_h */
