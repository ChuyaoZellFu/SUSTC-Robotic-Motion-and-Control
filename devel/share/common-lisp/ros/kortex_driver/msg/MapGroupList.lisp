; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude MapGroupList.msg.html

(cl:defclass <MapGroupList> (roslisp-msg-protocol:ros-message)
  ((map_groups
    :reader map_groups
    :initarg :map_groups
    :type (cl:vector kortex_driver-msg:MapGroup)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:MapGroup :initial-element (cl:make-instance 'kortex_driver-msg:MapGroup))))
)

(cl:defclass MapGroupList (<MapGroupList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapGroupList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapGroupList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<MapGroupList> is deprecated: use kortex_driver-msg:MapGroupList instead.")))

(cl:ensure-generic-function 'map_groups-val :lambda-list '(m))
(cl:defmethod map_groups-val ((m <MapGroupList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:map_groups-val is deprecated.  Use kortex_driver-msg:map_groups instead.")
  (map_groups m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapGroupList>) ostream)
  "Serializes a message object of type '<MapGroupList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'map_groups))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'map_groups))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapGroupList>) istream)
  "Deserializes a message object of type '<MapGroupList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'map_groups) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'map_groups)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:MapGroup))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapGroupList>)))
  "Returns string type for a message object of type '<MapGroupList>"
  "kortex_driver/MapGroupList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapGroupList)))
  "Returns string type for a message object of type 'MapGroupList"
  "kortex_driver/MapGroupList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapGroupList>)))
  "Returns md5sum for a message object of type '<MapGroupList>"
  "60049e6988ccab2e80e3d3cdf02a81da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapGroupList)))
  "Returns md5sum for a message object of type 'MapGroupList"
  "60049e6988ccab2e80e3d3cdf02a81da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapGroupList>)))
  "Returns full string definition for message of type '<MapGroupList>"
  (cl:format cl:nil "~%MapGroup[] map_groups~%================================================================================~%MSG: kortex_driver/MapGroup~%~%MapGroupHandle group_handle~%string name~%MappingHandle related_mapping_handle~%MapGroupHandle parent_group_handle~%MapGroupHandle[] children_map_group_handles~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapGroupList)))
  "Returns full string definition for message of type 'MapGroupList"
  (cl:format cl:nil "~%MapGroup[] map_groups~%================================================================================~%MSG: kortex_driver/MapGroup~%~%MapGroupHandle group_handle~%string name~%MappingHandle related_mapping_handle~%MapGroupHandle parent_group_handle~%MapGroupHandle[] children_map_group_handles~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapGroupList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'map_groups) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapGroupList>))
  "Converts a ROS message object to a list"
  (cl:list 'MapGroupList
    (cl:cons ':map_groups (map_groups msg))
))
