; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude MapGroup.msg.html

(cl:defclass <MapGroup> (roslisp-msg-protocol:ros-message)
  ((group_handle
    :reader group_handle
    :initarg :group_handle
    :type kortex_driver-msg:MapGroupHandle
    :initform (cl:make-instance 'kortex_driver-msg:MapGroupHandle))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (related_mapping_handle
    :reader related_mapping_handle
    :initarg :related_mapping_handle
    :type kortex_driver-msg:MappingHandle
    :initform (cl:make-instance 'kortex_driver-msg:MappingHandle))
   (parent_group_handle
    :reader parent_group_handle
    :initarg :parent_group_handle
    :type kortex_driver-msg:MapGroupHandle
    :initform (cl:make-instance 'kortex_driver-msg:MapGroupHandle))
   (children_map_group_handles
    :reader children_map_group_handles
    :initarg :children_map_group_handles
    :type (cl:vector kortex_driver-msg:MapGroupHandle)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:MapGroupHandle :initial-element (cl:make-instance 'kortex_driver-msg:MapGroupHandle)))
   (map_handles
    :reader map_handles
    :initarg :map_handles
    :type (cl:vector kortex_driver-msg:MapHandle)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:MapHandle :initial-element (cl:make-instance 'kortex_driver-msg:MapHandle)))
   (application_data
    :reader application_data
    :initarg :application_data
    :type cl:string
    :initform ""))
)

(cl:defclass MapGroup (<MapGroup>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapGroup>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapGroup)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<MapGroup> is deprecated: use kortex_driver-msg:MapGroup instead.")))

(cl:ensure-generic-function 'group_handle-val :lambda-list '(m))
(cl:defmethod group_handle-val ((m <MapGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:group_handle-val is deprecated.  Use kortex_driver-msg:group_handle instead.")
  (group_handle m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <MapGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:name-val is deprecated.  Use kortex_driver-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'related_mapping_handle-val :lambda-list '(m))
(cl:defmethod related_mapping_handle-val ((m <MapGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:related_mapping_handle-val is deprecated.  Use kortex_driver-msg:related_mapping_handle instead.")
  (related_mapping_handle m))

(cl:ensure-generic-function 'parent_group_handle-val :lambda-list '(m))
(cl:defmethod parent_group_handle-val ((m <MapGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:parent_group_handle-val is deprecated.  Use kortex_driver-msg:parent_group_handle instead.")
  (parent_group_handle m))

(cl:ensure-generic-function 'children_map_group_handles-val :lambda-list '(m))
(cl:defmethod children_map_group_handles-val ((m <MapGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:children_map_group_handles-val is deprecated.  Use kortex_driver-msg:children_map_group_handles instead.")
  (children_map_group_handles m))

(cl:ensure-generic-function 'map_handles-val :lambda-list '(m))
(cl:defmethod map_handles-val ((m <MapGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:map_handles-val is deprecated.  Use kortex_driver-msg:map_handles instead.")
  (map_handles m))

(cl:ensure-generic-function 'application_data-val :lambda-list '(m))
(cl:defmethod application_data-val ((m <MapGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:application_data-val is deprecated.  Use kortex_driver-msg:application_data instead.")
  (application_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapGroup>) ostream)
  "Serializes a message object of type '<MapGroup>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'group_handle) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'related_mapping_handle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'parent_group_handle) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'children_map_group_handles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'children_map_group_handles))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'map_handles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'map_handles))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'application_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'application_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapGroup>) istream)
  "Deserializes a message object of type '<MapGroup>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'group_handle) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'related_mapping_handle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'parent_group_handle) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'children_map_group_handles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'children_map_group_handles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:MapGroupHandle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'map_handles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'map_handles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:MapHandle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'application_data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'application_data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapGroup>)))
  "Returns string type for a message object of type '<MapGroup>"
  "kortex_driver/MapGroup")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapGroup)))
  "Returns string type for a message object of type 'MapGroup"
  "kortex_driver/MapGroup")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapGroup>)))
  "Returns md5sum for a message object of type '<MapGroup>"
  "f19116dedc7aefc01a2fc92e64e79046")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapGroup)))
  "Returns md5sum for a message object of type 'MapGroup"
  "f19116dedc7aefc01a2fc92e64e79046")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapGroup>)))
  "Returns full string definition for message of type '<MapGroup>"
  (cl:format cl:nil "~%MapGroupHandle group_handle~%string name~%MappingHandle related_mapping_handle~%MapGroupHandle parent_group_handle~%MapGroupHandle[] children_map_group_handles~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapGroup)))
  "Returns full string definition for message of type 'MapGroup"
  (cl:format cl:nil "~%MapGroupHandle group_handle~%string name~%MappingHandle related_mapping_handle~%MapGroupHandle parent_group_handle~%MapGroupHandle[] children_map_group_handles~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapGroup>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'group_handle))
     4 (cl:length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'related_mapping_handle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'parent_group_handle))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'children_map_group_handles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'map_handles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:length (cl:slot-value msg 'application_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapGroup>))
  "Converts a ROS message object to a list"
  (cl:list 'MapGroup
    (cl:cons ':group_handle (group_handle msg))
    (cl:cons ':name (name msg))
    (cl:cons ':related_mapping_handle (related_mapping_handle msg))
    (cl:cons ':parent_group_handle (parent_group_handle msg))
    (cl:cons ':children_map_group_handles (children_map_group_handles msg))
    (cl:cons ':map_handles (map_handles msg))
    (cl:cons ':application_data (application_data msg))
))
