; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude MappingList.msg.html

(cl:defclass <MappingList> (roslisp-msg-protocol:ros-message)
  ((mappings
    :reader mappings
    :initarg :mappings
    :type (cl:vector kortex_driver-msg:Mapping)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:Mapping :initial-element (cl:make-instance 'kortex_driver-msg:Mapping))))
)

(cl:defclass MappingList (<MappingList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MappingList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MappingList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<MappingList> is deprecated: use kortex_driver-msg:MappingList instead.")))

(cl:ensure-generic-function 'mappings-val :lambda-list '(m))
(cl:defmethod mappings-val ((m <MappingList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:mappings-val is deprecated.  Use kortex_driver-msg:mappings instead.")
  (mappings m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MappingList>) ostream)
  "Serializes a message object of type '<MappingList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mappings))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'mappings))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MappingList>) istream)
  "Deserializes a message object of type '<MappingList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mappings) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mappings)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:Mapping))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MappingList>)))
  "Returns string type for a message object of type '<MappingList>"
  "kortex_driver/MappingList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MappingList)))
  "Returns string type for a message object of type 'MappingList"
  "kortex_driver/MappingList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MappingList>)))
  "Returns md5sum for a message object of type '<MappingList>"
  "bb960bc69e45bc26dd9e7ed2dbaeeaaf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MappingList)))
  "Returns md5sum for a message object of type 'MappingList"
  "bb960bc69e45bc26dd9e7ed2dbaeeaaf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MappingList>)))
  "Returns full string definition for message of type '<MappingList>"
  (cl:format cl:nil "~%Mapping[] mappings~%================================================================================~%MSG: kortex_driver/Mapping~%~%MappingHandle handle~%string name~%uint32 controller_identifier~%MapGroupHandle active_map_group_handle~%MapGroupHandle[] map_group_handles~%MapHandle active_map_handle~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MappingList)))
  "Returns full string definition for message of type 'MappingList"
  (cl:format cl:nil "~%Mapping[] mappings~%================================================================================~%MSG: kortex_driver/Mapping~%~%MappingHandle handle~%string name~%uint32 controller_identifier~%MapGroupHandle active_map_group_handle~%MapGroupHandle[] map_group_handles~%MapHandle active_map_handle~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MappingList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mappings) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MappingList>))
  "Converts a ROS message object to a list"
  (cl:list 'MappingList
    (cl:cons ':mappings (mappings msg))
))
