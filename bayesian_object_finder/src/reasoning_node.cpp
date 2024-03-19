#include <ros/ros.h>
#include <iostream>
#include <string>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <bayesian_object_finder/UpdateObjectList.h>

#include <bayesian_object_finder/GetParentClass.h>

using namespace std;

class Reasoner
{
private:
    PrologClient pl_;
    int ID_;

    std::string srv_assert_knowledge_name_;
    ros::ServiceServer assert_knowledge_srv_;                            // Advertise service to assert knowledge in the ontology

    std::string srv_get_parent_name_;
    ros::ServiceServer get_parent_srv_;                            // Advertise service to assert knowledge in the ontology
public:

    Reasoner(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");

        if(pl_.waitForServer())
            pl_ = PrologClient("/rosprolog", true);

        ID_=0; //Global variable to include in the asserted instances

        srv_assert_knowledge_name_ = "assert_knowledge";
        assert_knowledge_srv_ = nh.advertiseService(srv_assert_knowledge_name_, &Reasoner::srv_assert_callback, this);

        srv_get_parent_name_ = "get_parent";
        get_parent_srv_ = nh.advertiseService(srv_get_parent_name_, &Reasoner::srv_parent_callback, this);
    };

    ~Reasoner(){

    };

private:


     /**
     * @brief Callback function for the service that adds objects to the map_objects list
     *
     * @param Request requested object to be added to the knowledge base
     * @param Respose response from the service when the object has been asserted (true/false)
     */
    bool srv_assert_callback(bayesian_object_finder::UpdateObjectList::Request &req,
                             bayesian_object_finder::UpdateObjectList::Response &res)
    {
        ROS_INFO_STREAM("Got new object: " << req.object_name);
        std::string object;

        //TODO: A2.T03: Modify this callback function to first verify that the seen object has a class, then the seen object can be asserted into the knowledge base. The response of this function is true if the assertion of knowledge is succesful (1.5 pts).

        object=req.object_name;
        getClass(object);

        res.confirmation = assertKnowledge(object);


        return res.confirmation;
    }

     /**
     * @brief Callback function for the service that adds objects to the map_objects list
     *
     * @param Request requested object to be added to the knowledge base
     * @param Respose response from the service when the object has been asserted (true/false)
     */
    bool srv_parent_callback(bayesian_object_finder::GetParentClass::Request &req,
                             bayesian_object_finder::GetParentClass::Response &res)
    {
        ROS_INFO_STREAM("Determining most likely location of: " << req.object_name);
        std::string object, parent_class;

        //TODO: A2.T03: Modify this callback function to first verify that the seen object has a class, then the seen object can be asserted into the knowledge base. The response of this function is true if the assertion of knowledge is succesful (1.5 pts).

        object=req.object_name;

        std::string query = "get_object_parent('"+object+"', X)";

        ROS_INFO_STREAM("query: "<<query);

        PrologQuery bdgs = pl_.query(query);

        for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
        {
            for (auto val : *it)
            {
                //A2.T03: Retrive the value from Prolog -> 1 pt
                parent_class = val.second.toString();
                ROS_WARN_STREAM("Parent class name: "<<parent_class);
                break;
            }
        }

        res.confirmation = true;
        res.parent = parent_class;

        return res.confirmation;
    }


    void getClass(std::string className)
    {

        // A2.T03:Save the query you want to ask Prolog into the variable "query", this variable is the prolog predicate that we define in the file "instance_utils" -> 1.5 pts
        std::string query= "get_class('"+className+"')";

        ROS_INFO_STREAM("query: "<<query);

        PrologQuery bdgs = pl_.query(query);

        bool res = false;
        for (auto &it : bdgs)
        {
            res = true;
            ROS_INFO_STREAM("A new class was created in the ontology");
            break;
        }


    }

    bool assertKnowledge(std::string className)
    {
        std::string instanceName;
        bool result = false;
        // A2.T03: Save the query you want to ask Prolog into the variable "query", this variable is the prolog predicate that we define in the file "instance_utils" -> 1 pt
        std::string check_if_class_exists = "rdf_has(ssy236Ontology:'"+className+"', rdf:type, owl:'Class')";
        std::string create_instance = "create_instance_from_class('"+className+"', '"+std::to_string(ID_)+"', Inst)";
        std::string query= check_if_class_exists+","+create_instance+".";

        ROS_INFO_STREAM("query: "<< query);

        PrologQuery bdgs = pl_.query(query);

        for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); it++)
        {
            for (auto val : *it)
            {
                //A2.T03: Retrive the value from Prolog -> 1 pt
                instanceName = val.second.toString();
                ROS_WARN_STREAM("new instance in knowledge base: "<<instanceName);
            }

        }

        bdgs = pl_.query(
            "owl_individual_of(ssy236Ontology:'"+className+"_"+std::to_string(ID_)+"', ssy236Ontology:'"+className+"')");
        for (auto &it : bdgs) {
            result = true;
            break;
        }

        bdgs.finish();

        ID_++;

        return result;
    }


}; //class Reasoner

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "reasoning_node");

    ros::NodeHandle nh;

    Reasoner myReasoner(nh);
    ros::spin();


    return 0;
}
