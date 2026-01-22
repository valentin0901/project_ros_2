import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler

def main():
    rclpy.init()
    nav = BasicNavigator()

    # 1. Attendre que Nav2 soit totalement prêt
    # (Lifecycle nodes, planner, controller, etc.)
    nav.waitUntilNav2Active()

    # 2. Définir une pose cible (Goal Pose)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()

    # --- MODIFIEZ CES COORDONNÉES ---
    goal_pose.pose.position.x = 1.5
    goal_pose.pose.position.y = 0.5
    
    # Orientation : On convertit un angle en degrés (ex: 90°) vers un Quaternion
    q = quaternion_from_euler(0, 0, 1.57) # 1.57 rad = 90 deg
    goal_pose.pose.orientation.x = q[0]
    goal_pose.pose.orientation.y = q[1]
    goal_pose.pose.orientation.z = q[2]
    goal_pose.pose.orientation.w = q[3]
    # --------------------------------

    # 3. Envoyer le robot à la destination
    print(f"En route vers : x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}...")
    nav.goToPose(goal_pose)

    # 4. Suivre la progression
    i = 0
    while not nav.isTaskComplete():
        i += 1
        feedback = nav.getFeedback()
        if feedback and i % 5 == 0:
            print(f"Distance restante : {feedback.distance_remaining:.2f} mètres.")

    # 5. Vérifier le résultat final
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Objectif atteint avec succès !")
    elif result == TaskResult.CANCELED:
        print("La mission a été annulée.")
    elif result == TaskResult.FAILED:
        print("L'objectif a échoué.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()