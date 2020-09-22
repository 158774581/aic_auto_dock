#include <aic_auto_dock/math/footprint.h>

std::vector<geometry_msgs::Point> makeFootprintFromParams(ros::NodeHandle& nh)
{
  std::string full_param_name;
  std::string full_radius_param_name;
  std::vector<geometry_msgs::Point> points;

  if (nh.searchParam("/move_base/local_costmap/footprint", full_param_name))
  {
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    nh.getParam(full_param_name, footprint_xmlrpc);
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString &&
        footprint_xmlrpc != "" && footprint_xmlrpc != "[]")
    {
      if (makeFootprintFromString(std::string(footprint_xmlrpc), points))
      {
        writeFootprintToParam(nh, points);
        return points;
      }
    }
    else if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      points = makeFootprintFromXMLRPC(footprint_xmlrpc, full_param_name);
      writeFootprintToParam(nh, points);
      return points;
    }
  }

  if (nh.searchParam("/move_base/local_costmap/robot_radius", full_radius_param_name))
  {
    double robot_radius;
    nh.param(full_radius_param_name, robot_radius, 1.234);
    points = makeFootprintFromRadius(robot_radius);
    nh.setParam("robot_radius", robot_radius);
  }
  // Else neither param was found anywhere this knows about, so
  // defaults will come from dynamic_reconfigure stuff, set in
  // cfg/Costmap2D.cfg and read in this file in reconfigureCB().
  return points;
}



bool makeFootprintFromString(const std::string& footprint_string, std::vector<geometry_msgs::Point>& footprint)
{
  std::string error;
  std::vector<std::vector<float> > vvf = parseVVF(footprint_string, error);

  if (error != "")
  {
    ROS_ERROR("Error parsing footprint parameter: '%s'", error.c_str());
    ROS_ERROR("  Footprint string was '%s'.", footprint_string.c_str());
    return false;
  }

  // convert vvf into points.
  if (vvf.size() < 3)
  {
    ROS_ERROR("You must specify at least three points for the robot footprint, reverting to previous footprint.");
    return false;
  }
  footprint.reserve(vvf.size());
  for (unsigned int i = 0; i < vvf.size(); i++)
  {
    if (vvf[ i ].size() == 2)
    {
      geometry_msgs::Point point;
      point.x = vvf[ i ][ 0 ];
      point.y = vvf[ i ][ 1 ];
      point.z = 0;
      footprint.push_back(point);
    }
    else
    {
      ROS_ERROR("Points in the footprint specification must be pairs of numbers.  Found a point with %d numbers.",
                 int(vvf[ i ].size()));
      return false;
    }
  }

  return true;
}

void writeFootprintToParam(ros::NodeHandle& nh, const std::vector<geometry_msgs::Point>& footprint)
{
  std::ostringstream oss;
  bool first = true;
  for (unsigned int i = 0; i < footprint.size(); i++)
  {
    geometry_msgs::Point p = footprint[ i ];
    if (first)
    {
      oss << "[[" << p.x << "," << p.y << "]";
      first = false;
    }
    else
    {
      oss << ",[" << p.x << "," << p.y << "]";
    }
  }
  oss << "]";
  nh.setParam("footprint", oss.str().c_str());
}


std::vector<geometry_msgs::Point> makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc,
                                const std::string& full_param_name)
{
  // Make sure we have an array of at least 3 elements.
  if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      footprint_xmlrpc.size() < 3)
  {
    ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
               full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
    throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                             "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }

  std::vector<geometry_msgs::Point> footprint;
  geometry_msgs::Point pt;

  for (int i = 0; i < footprint_xmlrpc.size(); ++i)
  {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        point.size() != 2)
    {
      ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                 full_param_name.c_str());
      throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x = getNumberFromXMLRPC(point[ 0 ], full_param_name);
    pt.y = getNumberFromXMLRPC(point[ 1 ], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

std::vector<geometry_msgs::Point> makeFootprintFromRadius(double radius)
{
  std::vector<geometry_msgs::Point> points;

  // Loop over 16 angles around a circle making a point each time
  int N = 16;
  geometry_msgs::Point pt;
  for (int i = 0; i < N; ++i)
  {
    double angle = i * 2 * M_PI / N;
    pt.x = cos(angle) * radius;
    pt.y = sin(angle) * radius;

    points.push_back(pt);
  }

  return points;
}

double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str());
    throw std::runtime_error("Values in the footprint specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

/** @brief Parse a vector of vector of floats from a string.
 * @param input
 * @param error_return
 * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...] */
std::vector<std::vector<float> > parseVVF(const std::string& input, std::string& error_return)
{
  std::vector<std::vector<float> > result;

  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<float> current_vector;
  while (!!input_ss && !input_ss.eof())
  {
    switch (input_ss.peek())
    {
    case EOF:
      break;
    case '[':
      depth++;
      if (depth > 2)
      {
        error_return = "Array depth greater than 2";
        return result;
      }
      input_ss.get();
      current_vector.clear();
      break;
    case ']':
      depth--;
      if (depth < 0)
      {
        error_return = "More close ] than open [";
        return result;
      }
      input_ss.get();
      if (depth == 1)
      {
        result.push_back(current_vector);
      }
      break;
    case ',':
    case ' ':
    case '\t':
      input_ss.get();
      break;
    default:  // All other characters should be part of the numbers.
      if (depth != 2)
      {
        std::stringstream err_ss;
        err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
        error_return = err_ss.str();
        return result;
      }
      float value;
      input_ss >> value;
      if (!!input_ss)
      {
        current_vector.push_back(value);
      }
      break;
    }
  }

  if (depth != 0)
  {
    error_return = "Unterminated vector string.";
  }
  else
  {
    error_return = "";
  }

  return result;
}
