/*
 * ActuatorGraphicsNode.cpp
 *
 *  Created on: 11 mars 2014
 *      Author: wla
 */
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <iostream>
#include <gtk/gtk.h>


namespace arp_hml
{

class ActuatorGraphicsNode
{
    public:
        GtkImage* m_leftCanonBall;

        ActuatorGraphicsNode(ros::NodeHandle &nh, int argc, char **argv)
        {
            //emergency_sub = nh.subscribe("/Protokrot/emergency_stop", 1, &HokuyoManager::auCallback, this);

            GtkBuilder *builder;
            GtkWidget *window;
            GError *error = NULL;

            /* Init GTK+ */
            gtk_init(&argc, &argv);

            /* Create new GtkBuilder object */
            builder = gtk_builder_new();
            /* Load UI from file. If error occurs, report it and quit application.
             * Replace "tut.glade" with your saved project. */
            if (!gtk_builder_add_from_file(builder, "ressource/glade/Actuators.glade", &error))
            {
                g_warning("%s", error->message);
                g_free(error);
                while(1);
            }

            /* Get main window pointer from UI */
            window = GTK_WIDGET(gtk_builder_get_object(builder, "Actuators"));
            m_leftCanonBall = GTK_IMAGE(gtk_builder_get_object(builder, "LeftCannonBall"));

            /* Connect signals */
            gtk_builder_connect_signals(builder, NULL);

            /* Destroy builder, since we don't need it anymore */
            g_object_unref(G_OBJECT(builder));

            /* Show window. All other widgets are automatically shown by GtkBuilder */
            gtk_widget_show(window);

        }

        ~ActuatorGraphicsNode()
        {
        }

        bool spin()
        {
            bool truc = false;
            /* Start main loop */
            if (truc)
                gtk_image_set_from_file(m_leftCanonBall, "ressource/glade/noball.png");
            else
                gtk_image_set_from_file(m_leftCanonBall, "ressource/glade/ball.png");

            truc = !truc;

            return gtk_main_iteration_do(false);
        }

        void shutDown()
        {
        }
};

}

arp_hml::ActuatorGraphicsNode * node;

void quit(int sig)
{
ROS_INFO("Ctrl C catched => Shuting down HokuyoManager");
node->shutDown();
ros::shutdown();
exit(0);
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "ActuatorGraphics");
ros::NodeHandle nh("ActuatorGraphics");
node = new arp_hml::ActuatorGraphicsNode(nh,argc,argv);
signal(SIGINT, quit);

ros::Rate r(10);

while (ros::ok() && node->spin())
{
    r.sleep();
    ros::spinOnce();
}

return true;
}

