/*
 * ActuatorsFrameGlade2014.cpp
 *
 *  Created on: Apr 15, 2014
 *      Author: ard
 */

#include "CannonFrame.hpp"

using namespace arp_hml;

CannonFrame::CannonFrame()
{
    m_Balls[LEFT]=
    {   NULL,NULL,NULL};
    m_Balls[RIGHT]=
    {   NULL,NULL,NULL};
}

CannonFrame::~CannonFrame()
{
}

bool CannonFrame::init(int argc, char **argv)
{
    GtkBuilder *builder = NULL;
    GtkWidget *window = NULL;
    GError *error = NULL;

    /* Init GTK+ */
    gtk_init(&argc, &argv);

    /* Create new GtkBuilder object */
    builder = gtk_builder_new();
    if (NULL == builder)
    {
        return false;
    }

    /* Load UI from file. If error occurs, report it and quit application.
     * Replace "tut.glade" with your saved project. */
    if (!gtk_builder_add_from_file(builder, "ressource/glade/Actuators.glade", &error))
    {
        g_warning("%s", error->message);
        g_free(error);
        return false;
    }

    /* Get main window pointer from UI */
    window = GTK_WIDGET(gtk_builder_get_object(builder, "Actuators"));
    if (NULL == window)
    {
        return false;
    }

    /* Initialize class members */
    if (!initCannon(LEFT, builder) || !initCannon(RIGHT, builder))
    {
        return false;
    }
    setNumberOfBallsInCanon(LEFT, 0);
    setStockerPosition(LEFT,IDLE);
    setFingerPosition(LEFT,DOWN);

    setNumberOfBallsInCanon(RIGHT, 0);
    setStockerPosition(RIGHT,IDLE);
    setFingerPosition(RIGHT,DOWN);

    /* Connect signals */
    gtk_builder_connect_signals(builder, NULL);

    /* Destroy builder, since we don't need it anymore */
    g_object_unref(G_OBJECT(builder));

    /* Show window. All other widgets are automatically shown by GtkBuilder */
    gtk_widget_show(window);

    return true;
}

bool CannonFrame::initCannon(eCanonSide side, GtkBuilder *builder)
{
    if (side == LEFT)
    {
        m_Balls[side][0] = GTK_IMAGE(gtk_builder_get_object(builder, "LeftBallBot"));
        m_Balls[side][1] = GTK_IMAGE(gtk_builder_get_object(builder, "LeftBallMid"));
        m_Balls[side][2] = GTK_IMAGE(gtk_builder_get_object(builder, "LeftBallTop"));
        m_Fingers[side] = GTK_IMAGE(gtk_builder_get_object(builder, "LeftFinger"));
        m_Stockers[side] = GTK_IMAGE(gtk_builder_get_object(builder, "LeftStocker"));

    }
    else
    {
        m_Balls[side][0] = GTK_IMAGE(gtk_builder_get_object(builder, "RightBallBot"));
        m_Balls[side][1] = GTK_IMAGE(gtk_builder_get_object(builder, "RightBallMid"));
        m_Balls[side][2] = GTK_IMAGE(gtk_builder_get_object(builder, "RightBallTop"));
        m_Fingers[side] = GTK_IMAGE(gtk_builder_get_object(builder, "RightFinger"));
        m_Stockers[side] = GTK_IMAGE(gtk_builder_get_object(builder, "RightStocker"));
    }

    if (NULL == m_Balls[side][0] || NULL == m_Balls[side][1] || NULL == m_Balls[side][2])
    {
        return false;
    }

    return true;
}

bool CannonFrame::spin()
{
    return gtk_main_iteration_do(false);
}

void CannonFrame::shutDown()
{
}

void CannonFrame::setNumberOfBallsInCanon(eCanonSide side, int nbBalls)
{
    switch (nbBalls)
    {
        case 0:
        default:
            gtk_image_set_from_file(m_Balls[side][0], "ressource/glade/noball.png");
            gtk_image_set_from_file(m_Balls[side][1], "ressource/glade/noball.png");
            gtk_image_set_from_file(m_Balls[side][2], "ressource/glade/noball.png");
            break;

        case 1:
            gtk_image_set_from_file(m_Balls[side][0], "ressource/glade/ball.png");
            gtk_image_set_from_file(m_Balls[side][1], "ressource/glade/noball.png");
            gtk_image_set_from_file(m_Balls[side][2], "ressource/glade/noball.png");
            break;

        case 2:
            gtk_image_set_from_file(m_Balls[side][0], "ressource/glade/ball.png");
            gtk_image_set_from_file(m_Balls[side][1], "ressource/glade/ball.png");
            gtk_image_set_from_file(m_Balls[side][2], "ressource/glade/noball.png");
            break;

        case 3:
            gtk_image_set_from_file(m_Balls[side][0], "ressource/glade/ball.png");
            gtk_image_set_from_file(m_Balls[side][1], "ressource/glade/ball.png");
            gtk_image_set_from_file(m_Balls[side][2], "ressource/glade/ball.png");
            break;

    }
}

void CannonFrame::setStockerPosition(eCanonSide side, enum eStockerPosition pos)
{
    switch (pos)
    {
        case LOADING:
        default:
            gtk_image_set_from_file(m_Stockers[side], "ressource/glade/stock_loading.png");
            break;

        case IDLE:
            gtk_image_set_from_file(m_Stockers[side], "ressource/glade/stock_idle.png");
            break;

        case UNLOADING:
            gtk_image_set_from_file(m_Stockers[side], "ressource/glade/stock_unloading.png");
            break;
    }
}

void CannonFrame::setFingerPosition(eCanonSide side, enum eFingerPosition pos)
{
    switch (pos)
    {
        case DOWN:
        default:
            gtk_image_set_from_file(m_Fingers[side], "ressource/glade/doigt_bas.png");
            break;

        case UP:
            gtk_image_set_from_file(m_Fingers[side], "ressource/glade/doigt_haut.png");
            break;

        case ARMED:
            gtk_image_set_from_file(m_Fingers[side], "ressource/glade/doigt_arme.png");
            break;

        case SHOOTING:
            gtk_image_set_from_file(m_Fingers[side], "ressource/glade/doigt_shoot.png");
            break;
    }
}

