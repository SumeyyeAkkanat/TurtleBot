#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <sstream>
#include "geometry.h"
#include "pid.h"


// global deðiþkenler
tf::Point Odom_pos;    //odometry pozisyonu (x, y, z)
double Odom_yaw;    //odometry oryantasyonu (yaw)
double Odom_v, Odom_w;    //odometry doðrusal ve açýsal hýzlar

// ROS Topic Publishers
ros::Publisher cmd_vel_pub;
ros::Publisher marker_pub;

// ROS Topic Subscribers
ros::Subscriber odom_sub;


//PID için global deðiþkenler

int num_slice2 = 50;               // daireyi parçalara ayýrma

double maxSpeed = 0.5;
double distanceConst = 0.5;
double dt = 0.1, maxT = M_PI, minT = -M_PI, Kp = 0.3, Ki = 0.05, Kd = 0.01;
double dtS = 0.1, maxS = maxSpeed, minS = 0.0, KpS = 0.08, KiS = 0.01, KdS = 0.005;


double getDistance(Point &p1, Point &p2);


 // odom_sub için kullanýlan, Odometri msg için callback fonksiyonu

void odomCallback(const nav_msgs::Odometry odom_msg) {
      // odom msg 'duyulduðunda' pozisyon ve oryantasyon (yaw-sapma) bilgileri alýnýr.
     //tf dökümantasyonu: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
     // pointMsgToTF: Point msg'yi Point'e dönüþtürme

    tf::pointMsgToTF(odom_msg.pose.pose.position, Odom_pos);
    Odom_yaw = tf::getYaw(odom_msg.pose.pose.orientation);

    //Gözlemlenen doðrusal ve aaçýsal hýzlarý güncelleme (simülasyondan yayýnlanan gerçek hýzlar)
    Odom_v = odom_msg.twist.twist.linear.x;
    Odom_w = odom_msg.twist.twist.angular.z;

    //terminal ekranýnda göstermek için: 
    //ROS_INFO("Position: (%f, %f); Yaw: %f", Odom_pos.x(), Odom_pos.y(), Odom_yaw);
}


/*
 * Fonksiyon ile Rviz'de dairesel bir þerit çizen görüntüleme fonksiyonu
              (x+0.5)^2 + (y-1)^2 = 4^2 
 */
void displayLane(bool isTrajectoryPushed, Geometry &geometry) {
    static visualization_msgs::Marker path;
    path.type = visualization_msgs::Marker::LINE_STRIP;

    path.header.frame_id = "odom";  
	//NOT: bu, Rviz'deki sabit çerçeve kimliði giriþine "eþleþtirilmelidir", Rviz'de tb3-sahte için varsayýlan ayar "odom" dur. Herhangi bir sorun yoksa bu satýrýn olduðu gibi kalmasý gerekir.
    path.header.stamp = ros::Time::now();
    path.ns = "odom";
    path.id = 0;
    path.action = visualization_msgs::Marker::ADD; // çizgi iþaretleyici kullanma
    path.lifetime = ros::Duration();

    // yol çizgisi þeridi mavi
    path.color.b = 1.0;
    path.color.a = 1.0;

    path.scale.x = 0.02;
    path.pose.orientation.w = 1.0;


    static int slice_index2 = 0;


    VECTOR2D *prev = NULL, *current = NULL;

    while (path.points.size() <= num_slice2) {
        geometry_msgs::Point p;

        float angle = slice_index2 * 2 * M_PI / num_slice2;
        slice_index2++;
        p.x = 4 * cos(angle) - 0.5;       //some random circular trajectory, with radius 4, and offset (-0.5, 1, 0)
        p.y = 4 * sin(angle) + 1.0;
        p.z = 0;

        path.points.push_back(p);         //çizgi þerit tipi olan çizim yolu için


        //Yalnýzca 1. uygulamada PID kullanýmý için puan eklenir
        if (!isTrajectoryPushed) {

            VECTOR2D *temp = new VECTOR2D(p.x, p.y);

            geometry.trajectory.push_back(*temp);

            current = temp;

            if (prev != NULL) {

                geometry.path.push_back(geometry.getLineSegment(*prev, *current));

            }
            prev = current;

        }

    }

    //Baþlangýç ve bitiþ noktalarýnýn baðlanmasý
    if (prev != NULL && current != NULL && current != prev)
        geometry.path.push_back(geometry.getLineSegment(*prev, *current));


    marker_pub.publish(path);
}


// Ana Fonksiyon
int main(int argc, char **argv) {

    ros::init(argc, argv, "control");
    ros::NodeHandle n("~");
    tf::TransformListener m_listener;
    tf::StampedTransform transform;

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    odom_sub = n.subscribe("odom", 10, odomCallback);

    ros::Rate loop_rate(10); // ros saniyede 10 frame döndürür

    //turtlebot'a geçmek için kontrol girdilerimizi de belirten doðrusa ve açýsal hýzlarýmýzý (v,w) belirtmek için  geometry_msgs::Twist kullanýyoruz
    geometry_msgs::Twist tw_msg;


    //trajectory details are here
    Geometry geometry;

    double angleError = 0.0;
    double speedError = 0.0;

    int frame_count = 0;
    PID pidTheta = PID(dt, maxT, minT, Kp, Kd, Ki);
    PID pidVelocity = PID(dtS, maxS, minS, KpS, KdS, KiS);
    
    while (ros::ok()) {
        if (frame_count == 0)
            displayLane(false, geometry);
        else
            displayLane(true, geometry);
        //ROS_INFO("frame %d", frame_count);

/*

KONTROL STRATEJÝSÝ
Öncelikle araç dinamikleri belirlenir. (dubins car model)
Kontrol giriþi hesaplanýr (açýsal hýz) w, w deðeri ----> "tw_msg.angular.z"  iletilir.

*/

        double omega = 0.0;
        double speed = 0.0;
        double prevDistError = 1000.0;


        double tb3_lenth = 0.125;

        // Hata Hesaplama
        VECTOR2D current_pos, pos_error;
        current_pos.x = Odom_pos.x();
        current_pos.y = Odom_pos.y();
        //ROS_INFO("Nearest %f,%f ", current_pos.x, current_pos.y);

        Geometry::LineSegment *linesegment = geometry.getNearestLine(current_pos);

        Geometry::LineSegment linesegmentPerpen = geometry.getMinimumDistanceLine(*linesegment, current_pos);

        //Get Next LineSegment to do velocity PID

        Geometry::LineSegment *nextLinesegment = geometry.getNextLineSegment(linesegment);

        double targetDistanceEnd = geometry.getDistance(current_pos, linesegment->endP);
        double targetDistanceStart = geometry.getDistance(current_pos, linesegment->startP);

        //Distance Error
        double distError = 0.0;

        double targetAnglePerpen = geometry.getGradient(current_pos, linesegmentPerpen.endP);

        VECTOR2D target = linesegment->endP;
        double targetAngle = geometry.getGradient(current_pos, target);
        double distanceToClosestPath = abs(linesegment->disatanceToAObj);

        //Açýlara göre hata hesaplamasý
        if (distanceToClosestPath < distanceConst) {

            // Bu, çizgi parçasýnýn son noktasýna doðru gider-> Select vary small distanceConst

            //angleError = targetAngle - Odom_yaw;
            double directional = targetAngle;

            double discripancy = targetAnglePerpen - directional;
            discripancy = geometry.correctAngle(discripancy);

            //Hataya bir miktar dik açý potion'ý eklemek
            discripancy = 0.5* discripancy / distanceConst * abs(distanceToClosestPath);

            double combined = targetAngle + discripancy;

            angleError = combined - Odom_yaw;


        } else {

            //Bu, yolun minimum mesafesine doðru gider

            angleError = targetAnglePerpen - Odom_yaw;

        }

        speed = maxSpeed;

//Çizgiler uzun ve kenarlarý keskinse
        if (nextLinesegment->disatance > 3.0 && linesegment->disatance > 3.0) {
            //angleError keskin dönüþler için düzeltme
            if (targetDistanceEnd < 0.5) {
                double futureAngleChange = nextLinesegment->gradient - linesegment->gradient;
                futureAngleChange = geometry.correctAngle(futureAngleChange);

                //Hataya bir miktar dik açý potion'ý eklemek
                futureAngleChange = futureAngleChange / distanceConst * abs(targetDistanceEnd);

                double combined = targetAngle + futureAngleChange;

                angleError = combined - Odom_yaw;
            }

            //Keskin dönüþler için hýz hatasý hesaplama
            if (targetDistanceStart < 0.7 || targetDistanceEnd < 0.7) {

                double targetDistance = targetDistanceEnd;

                if (targetDistanceStart < targetDistanceEnd)
                    targetDistance = targetDistanceStart;

                double speedError = 0.3 * maxSpeed * exp(-abs(targetDistance));

                speed = pidVelocity.calculate(maxSpeed, -speedError);
            }

        }

        //Büyük açýlar için hata açý düzeltme
        angleError = geometry.correctAngle(angleError);
        //Açý için PID
        omega = pidTheta.calculate(0, -angleError);

        //ROS_INFO("Nearest %f,%f, dist %f ,ShortestDistanceVecAngle %f, Odom_yaw %f, Error: %f , omega: %f", linesegment->startP.x,linesegment->startP.y, linesegment->disatanceToAObj,angleError,Odom_yaw,angleError,omega);

        ROS_INFO("Odom_yaw %f, Angle Error: %f , omega: %f Speed %f, Speed Error: %f , speedSet: %f", Odom_yaw,
                 angleError, omega, Odom_v, speedError, speed);


        //doðrusal hýz için, 'v' hýzýný temsil etmek için yalnýzca 3B doðrusal hýzýn "linear.x" ilk bileþenini kullanýrýz. 
        tw_msg.linear.x = speed;
        //açýsal hýz için, "w" hýzýný (radyan cinsinden) temsil etmek için yalnýzca 3B açýsal hýzýn üçüncü bileþenini "angular.z" kullanýrýz.
        tw_msg.angular.z = omega;

        //Bu mesajý robota yayýnlama
        cmd_vel_pub.publish(tw_msg);

        frame_count++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}

 // Bu fonksiyon, verilen 2 nokta arasýndaki öklid mesafesini açýklar.
 // Hungry yapýlacaksa sqrt kaldýrýlýr.
double getDistance(Point &p1, Point &p2) {
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}


