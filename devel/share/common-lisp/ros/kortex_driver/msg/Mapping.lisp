; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Mapping.msg.html

(cl:defclass <Mapping> (roslisp-msg-protocol:ros-message)
  ((handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:MappingHandle
    :initform (cl:make-instance 'kortex_driver-msg:MappingHandle))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (controller_identifier
    :reader controller_identifier
    :initarg :controller_identifier
    :type cl:integer
    :initform 0)
   (active_map_group_handle
    :reader active_map_group_handle
    :initarg :active_map_group_handle
    :type kortex_driver-msg:MapGroupHandle
    :initform (cl:make-instance 'kortex_driver-msg:MapGroupHandle))
   (map_group_handles
    :reader map_group_handles
    :initarg :map_group_handles
    :type (cl:vector kortex_driver-msg:MapGroupHandle)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:MapGroupHandle :initial-element (cl:make-instance 'kortex_driver-msg:MapGroupHandle)))
   (active_map_handle
    :reader active_map_handle
    :initarg :active_map_handle
    :type kortex_driver-msg:MapHandle
    :initform (cl:make-instance 'kortex_driver-msg:MapHandle))
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

(cl:defclass Mapping (<Mapping>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Mapping>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Mapping)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Mapping> is deprecated: use kortex_driver-msg:Mapping instead.")))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <Mapping>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Mapping>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:name-val is deprecated.  Use kortex_driver-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'controller_identifier-val :lambda-list '(m))
(cl:defmethod controller_identifier-val ((m <Mapping>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:controller_identifier-val is deprecated.  Use kortex_driver-msg:controller_identifier instead.")
  (controller_identifier m))

(cl:ensure-generic-function 'active_map_group_handle-val :lambda-list '(m))
(cl:defmethod active_map_group_handle-val ((m <Mapping>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:active_map_group_handle-val is deprecated.  Use kortex_driver-msg:active_map_group_handle instead.")
  (active_map_group_handle m))

(cl:ensure-generic-function 'map_group_handles-val :lambda-list '(m))
(cl:defmethod map_group_handles-val ((m <Mapping>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:map_group_handles-val is deprecated.  Use kortex_driver-msg:map_group_handles instead.")
  (map_group_handles m))

(cl:ensure-generic-function 'active_map_handle-val :lambda-list '(m))
(cl:defmethod active_map_handle-val ((m <Mapping>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:active_map_handle-val is deprecated.  Use kortex_driver-msg:active_map_handle instead.")
  (active_map_handle m))

(cl:ensure-generic-function 'map_handles-val :lambda-list '(m))
(cl:defmethod map_handles-val ((m <Mapping>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:map_handles-val is deprecated.  Use kortex_driver-msg:map_handles instead.")
  (map_handles m))

(cl:ensure-generic-function 'application_data-val :lambda-list '(m))
(cl:defmethod application_data-val ((m <Mapping>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:application_data-val is deprecated.  Use kortex_driver-msg:application_data instead.")
  (application_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Mapping>) ostream)
  "Serializes a message object of type '<Mapping>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'controller_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'controller_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'controller_identifier)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'controller_identifier)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'active_map_group_handle) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'map_group_handles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'map_group_handles))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'active_map_handle) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Mapping>) istream)
  "Deserializes a message object of type '<Mapping>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'controller_identifier)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'active_map_group_handle) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'map_group_handles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'map_group_handles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:MapGroupHandle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'active_map_handle) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Mapping>)))
  "Returns string type for a message object of type '<Mapping>"
  "kortex_driver/Mapping")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Mapping)))
  "Returns string type for a message object of type 'Mapping"
  "kortex_driver/Mapping")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Mapping>)))
  "Returns md5sum for a message object of type '<Mapping>"
  "fda005d3531fdfb82d459bb0078389d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Mapping)))
  "Returns md5sum for a message object of type 'Mapping"
  "fda005d3531fdfb82d459bb0078389d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Mapping>)))
  "Returns full string definition for message of type '<Mapping>"
  (cl:format cl:nil "~%MappingHandle handle~%string name~%uint32 controller_identifier~%MapGroupHandle active_map_group_handle~%MapGroupHandle[] map_group_handles~%MapHandle active_map_handle~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Mapping)))
  "Returns full string definition for message of type 'Mapping"
  (cl:format cl:nil "~%MappingHandle handle~%string name~%uint32 controller_identifier~%MapGroupHandle active_map_group_handle~%MapGroupHandle[] map_group_handles~%MapHandle active_map_handle~%MapHandle[] map_handles~%string application_data~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapGroupHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/MapHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Mapping>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     4 (cl:length (cl:slot-value msg 'name))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'active_map_group_handle))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'map_group_handles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'active_map_handle))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'map_handles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:length (cl:slot-value msg 'application_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Mapping>))
  "Converts a ROS message object to a list"
  (cl:list 'Mapping
    (cl:cons ':handle (handle msg))
    (cl:cons ':name (name msg))
    (cl:cons ':controller_identifier (controller_identifier msg))
    (cl:cons ':active_map_group_handle (active_map_group_handle msg))
    (cl:cons ':map_group_handles (map_group_handles msg))
    (cl:cons ':active_map_handle (active_map_handle msg))
    (cl:cons ':map_handles (map_handles msg))
    (cl:cons ':application_data (application_data msg))
))
