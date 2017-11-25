#if !defined(sim_sim_h)
#define sim_sim_h

#include "Framework/Test.h"

class Sim : public Test
{
public:
    Sim()
    {
        // ground
        b2Body* ground = NULL;
        {
            b2BodyDef bd;
            ground = m_world->CreateBody(&bd);
            b2EdgeShape shape;
            shape.Set(b2Vec2(-40, 0), b2Vec2(40, 0));
            ground->CreateFixture(&shape, 0.0);
        }

        // bowl
        b2Body* bowl_bottom = NULL;
        b2Body* bowl_left = NULL;
        b2Body* bowl_right = NULL;
        double l = 2;
        double w = 0.1;
        {
            b2BodyDef bd;
            bowl_bottom = m_world->CreateBody(&bd);
            b2PolygonShape shape;
            shape.SetAsBox(l, w, b2Vec2(0, w), 0);
            bowl_bottom->CreateFixture(&shape, 0);
        }
        {
            b2BodyDef bd;
            bowl_left = m_world->CreateBody(&bd);
            b2PolygonShape shape;
            shape.SetAsBox(0.5 * l, w,
                           b2Vec2(-(1.0 + 0.25 * sqrt(3)) * l, w + 0.25 * l),
                           -M_PI / 6.0);
            bowl_left->CreateFixture(&shape, 0);
        }
        {
            b2BodyDef bd;
            bowl_right = m_world->CreateBody(&bd);
            b2PolygonShape shape;
            shape.SetAsBox(0.5 * l, w,
                           b2Vec2((1.0 + 0.25 * sqrt(3)) * l, w + 0.25 * l),
                           M_PI / 6.0);
            bowl_right->CreateFixture(&shape, 0);
        }

        // arm
        b2Body* base = NULL;
        double bl = 2;        /* base length */
        double bw = 0.5;       /* base width */
        b2Vec2 base_pos(l, 6);
        {
            b2BodyDef bd;
            base = m_world->CreateBody(&bd);
            b2PolygonShape shape1;
            shape1.SetAsBox(0.5 * bw, 0.5 * bl,
                            base_pos + b2Vec2(0, 0.5 * bl),
                            0);
            base->CreateFixture(&shape1, 0);
            b2PolygonShape shape2;
            shape2.SetAsBox(0.5 * bl, 0.5 * bw,
                            base_pos + b2Vec2(0, bl - 0.5 * bw),
                            0);
            base->CreateFixture(&shape2, 0);
        }
        b2Body* upper = NULL;
        b2Body* lower = NULL;
        double al = 3;        /* length */
        double aw = 0.5;       /* width */
        double m = 1;
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position = base_pos + b2Vec2(-0.5 * al, 0);
            upper = m_world->CreateBody(&bd);
            b2PolygonShape shape;
            shape.SetAsBox(0.5 * al, 0.5 * aw);
            upper->CreateFixture(&shape, m);
            b2RevoluteJointDef rjd;
            rjd.Initialize(base, upper, base_pos);
            rjd.motorSpeed = 1.0f * b2_pi;
            rjd.maxMotorTorque = 10000.0f;
            rjd.enableMotor = false;
            rjd.lowerAngle = -0.8 * b2_pi;
            rjd.upperAngle = 0.8 * b2_pi;
            rjd.enableLimit = true;
            mpSh = (b2RevoluteJoint*)m_world->CreateJoint(&rjd);
        }
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position = base_pos + b2Vec2(-1.5 * al, 0);
            lower = m_world->CreateBody(&bd);
            b2PolygonShape shape;
            shape.SetAsBox(0.5 * al, 0.5 * aw);
            lower->CreateFixture(&shape, m);
            b2RevoluteJointDef rjd;
            rjd.Initialize(upper, lower, base_pos + b2Vec2(-1.0 * al, 0));
            rjd.motorSpeed = 1.0f * b2_pi;
            rjd.maxMotorTorque = 10000.0f;
            rjd.enableMotor = false;
            rjd.lowerAngle = -0.8 * b2_pi;
            rjd.upperAngle = 0.8 * b2_pi;
            rjd.enableLimit = true;
            mpEl = (b2RevoluteJoint*)m_world->CreateJoint(&rjd);
        }

        b2Body* hand = NULL;
        double hl = 1;
        double hw = 0.3;
        double hs = 2;          /* stride */
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position = base_pos + b2Vec2(-2.0 * al - 0.5 * hl, 0);
            hand = m_world->CreateBody(&bd);
            b2PolygonShape shape1;
            shape1.SetAsBox(0.5 * hl, 0.5 * hw);
            hand->CreateFixture(&shape1, 0.5 * m);
            b2PolygonShape shape2;
            shape2.SetAsBox(0.5 * hw, 0.5 * hs, b2Vec2(-0.5 * hl + 0.5 * hw, 0), 0);
            hand->CreateFixture(&shape2, 0.5 * m);
            b2RevoluteJointDef rjd;
            rjd.Initialize(lower, hand, base_pos + b2Vec2(-2.0 * al, 0));
            rjd.motorSpeed = 1.0f * b2_pi;
            rjd.maxMotorTorque = 10000.0f;
            rjd.enableMotor = false;
            rjd.lowerAngle = -0.8 * b2_pi;
            rjd.upperAngle = 0.8 * b2_pi;
            rjd.enableLimit = true;
            mpWr = (b2RevoluteJoint*)m_world->CreateJoint(&rjd);
        }

        b2Body* finger0 = NULL;
        b2Body* finger1 = NULL;
        double fl = 1;
        double fw = 0.3;
        double fm = 0.1;
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position = base_pos + b2Vec2(-2.0 * al - hl - 0.5 * fl, 0.25 * hs);
            finger0 = m_world->CreateBody(&bd);
            b2PolygonShape shape;
            shape.SetAsBox(0.5 * fl, 0.5 * fw);
            finger0->CreateFixture(&shape, fm);
            b2PrismaticJointDef pjd;
            pjd.Initialize(hand, finger0,
                           base_pos + b2Vec2(-2.0 * al - hl, 0.25 * hs),
                           b2Vec2(0.0, 1.0));
            pjd.motorSpeed = 1.0;
            pjd.maxMotorForce = 10000.0;
            pjd.enableMotor = false;
            pjd.lowerTranslation = -0.25 * hs;
            pjd.upperTranslation = 0.25 * hs;
            pjd.enableLimit = true;
            mpGr0 = (b2PrismaticJoint*)m_world->CreateJoint(&pjd);
        }
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position = base_pos + b2Vec2(-2.0 * al - hl - 0.5 * fl, -0.25 * hs);
            finger1 = m_world->CreateBody(&bd);
            b2PolygonShape shape;
            shape.SetAsBox(0.5 * fl, 0.5 * fw);
            finger1->CreateFixture(&shape, fm);
            b2PrismaticJointDef pjd;
            pjd.Initialize(hand, finger1,
                           base_pos + b2Vec2(-2.0 * al - hl, -0.25 * hs),
                           b2Vec2(0.0, 1.0));
            pjd.motorSpeed = 1.0;
            pjd.maxMotorForce = 10000.0;
            pjd.enableMotor = false;
            pjd.lowerTranslation = -0.25 * hs;
            pjd.upperTranslation = 0.25 * hs;
            pjd.enableLimit = true;
            mpGr1 = (b2PrismaticJoint*)m_world->CreateJoint(&pjd);
        }
    }

    void Keyboard(int key)
    {
        switch (key)
        {
            case GLFW_KEY_L:
                mpSh->EnableLimit(!mpSh->IsLimitEnabled());
                break;

            case GLFW_KEY_M:
                mpSh->EnableMotor(!mpSh->IsMotorEnabled());
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

    b2RevoluteJoint* mpSh;
    b2RevoluteJoint* mpEl;
    b2RevoluteJoint* mpWr;
    b2PrismaticJoint* mpGr0;
    b2PrismaticJoint* mpGr1;
};

#endif /* sim_sim_h */
