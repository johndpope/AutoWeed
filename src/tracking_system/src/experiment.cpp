void fromMsg(const geometry_msgs::TransformStamped& msg, tf2::Stamped<tf2::Transform>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  tf2::Transform tmp;
  fromMsg(msg.transform, tmp);
  out.setData(tmp);
}

void fromMsg(const geometry_msgs::Transform& in, tf2::Transform& out)
{
  tf2::Vector3 v;
  fromMsg(in.translation, v);
  out.setOrigin(v);
  // w at the end in the constructor
  tf2::Quaternion q;
  fromMsg(in.rotation, q);
  out.setRotation(q);
}

void fromMsg(const geometry_msgs::Vector3Stamped& msg, tf2::Stamped<tf2::Vector3>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  out.setData(tf2::Vector3(msg.vector.x, msg.vector.y, msg.vector.z));
}

void fromMsg(const geometry_msgs::Quaternion& in, tf2::Quaternion& out)
{
    // w at the end in the constructor
    out = tf2::Quaternion(in.x, in.y, in.z, in.w);
}

void fromMsg(const geometry_msgs::TransformStamped& msg, tf2::Stamped<tf2::Transform>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  tf2::Transform tmp;
  tmp.setOrigin(tf2::Vector3(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z));
  tmp.setRotation(tf2::Quaternion(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w));
  out.setData(tmp);
}
