; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CompleteProductConfiguration.msg.html

(cl:defclass <CompleteProductConfiguration> (roslisp-msg-protocol:ros-message)
  ((kin
    :reader kin
    :initarg :kin
    :type cl:string
    :initform "")
   (model
    :reader model
    :initarg :model
    :type cl:integer
    :initform 0)
   (country_code
    :reader country_code
    :initarg :country_code
    :type kortex_driver-msg:CountryCode
    :initform (cl:make-instance 'kortex_driver-msg:CountryCode))
   (assembly_plant
    :reader assembly_plant
    :initarg :assembly_plant
    :type cl:string
    :initform "")
   (model_year
    :reader model_year
    :initarg :model_year
    :type cl:string
    :initform "")
   (degree_of_freedom
    :reader degree_of_freedom
    :initarg :degree_of_freedom
    :type cl:integer
    :initform 0)
   (base_type
    :reader base_type
    :initarg :base_type
    :type cl:integer
    :initform 0)
   (end_effector_type
    :reader end_effector_type
    :initarg :end_effector_type
    :type cl:integer
    :initform 0)
   (vision_module_type
    :reader vision_module_type
    :initarg :vision_module_type
    :type cl:integer
    :initform 0)
   (interface_module_type
    :reader interface_module_type
    :initarg :interface_module_type
    :type cl:integer
    :initform 0)
   (arm_laterality
    :reader arm_laterality
    :initarg :arm_laterality
    :type cl:integer
    :initform 0)
   (wrist_type
    :reader wrist_type
    :initarg :wrist_type
    :type cl:integer
    :initform 0)
   (brake_type
    :reader brake_type
    :initarg :brake_type
    :type cl:integer
    :initform 0))
)

(cl:defclass CompleteProductConfiguration (<CompleteProductConfiguration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CompleteProductConfiguration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CompleteProductConfiguration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CompleteProductConfiguration> is deprecated: use kortex_driver-msg:CompleteProductConfiguration instead.")))

(cl:ensure-generic-function 'kin-val :lambda-list '(m))
(cl:defmethod kin-val ((m <CompleteProductConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:kin-val is deprecated.  Use kortex_driver-msg:kin instead.")
  (kin m))

(cl:ensure-generic-function 'model-val :lambda-list '(m))
(cl:defmethod model-val ((m <CompleteProductConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:model-val is deprecated.  Use kortex_driver-msg:model instead.")
  (model m))

(cl:ensure-generic-function 'country_code-val :lambda-list '(m))
(cl:defmethod country_code-val ((m <CompleteProductConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:country_code-val is deprecated.  Use kortex_driver-msg:country_code instead.")
  (country_code m))

(cl:ensure-generic-function 'assembly_plant-val :lambda-list '(m))
(cl:defmethod assembly_plant-val ((m <CompleteProductConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:assembly_plant-val is deprecated.  Use kortex_driver-msg:assembly_plant instead.")
  (assembly_plant m))

(cl:ensure-generic-function 'model_year-val :lambda-list '(m))
(cl:defmethod model_year-val ((m <CompleteProductConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:model_year-val is deprecated.  Use kortex_driver-msg:model_year instead.")
  (model_year m))

(cl:ensure-generic-function 'degree_of_freedom-val :lambda-list '(m))
(cl:defmethod degree_of_freedom-val ((m <CompleteProductConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:degree_of_freedom-val is deprecated.  Use kortex_driver-msg:degree_of_freedom instead.")
  (degree_of_freedom m))

(cl:ensure-generic-function 'base_type-val :lambda-list '(m))
(cl:defmethod base_type-val ((m <CompleteProductConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:base_type-val is deprecated.  Use kortex_driver-msg:base_type instead.")
  (base_type m))

(cl:ensure-generic-function 'end_effector_type-val :lambda-list '(m))
(cl:defmethod end_effector_type-val ((m <CompleteProductConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:end_effector_type-val is deprecated.  Use kortex_driver-msg:end_effector_type instead.")
  (end_effector_type m))

(cl:ensure-generic-function 'vision_module_type-val :lambda-list '(m))
(cl:defmethod vision_module_type-val ((m <CompleteProductConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:vision_module_type-val is deprecated.  Use kortex_driver-msg:vision_module_type instead.")
  (vision_module_type m))

(cl:ensure-generic-function 'interface_module_type-val :lambda-list '(m))
(cl:defmethod interface_module_type-val ((m <CompleteProductConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:interface_module_type-val is deprecated.  Use kortex_driver-msg:interface_module_type instead.")
  (interface_module_type m))

(cl:ensure-generic-function 'arm_laterality-val :lambda-list '(m))
(cl:defmethod arm_laterality-val ((m <CompleteProductConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:arm_laterality-val is deprecated.  Use kortex_driver-msg:arm_laterality instead.")
  (arm_laterality m))

(cl:ensure-generic-function 'wrist_type-val :lambda-list '(m))
(cl:defmethod wrist_type-val ((m <CompleteProductConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:wrist_type-val is deprecated.  Use kortex_driver-msg:wrist_type instead.")
  (wrist_type m))

(cl:ensure-generic-function 'brake_type-val :lambda-list '(m))
(cl:defmethod brake_type-val ((m <CompleteProductConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:brake_type-val is deprecated.  Use kortex_driver-msg:brake_type instead.")
  (brake_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CompleteProductConfiguration>) ostream)
  "Serializes a message object of type '<CompleteProductConfiguration>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'kin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'kin))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'model)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'model)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'model)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'model)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'country_code) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'assembly_plant))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'assembly_plant))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_year))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_year))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'degree_of_freedom)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'degree_of_freedom)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'degree_of_freedom)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'degree_of_freedom)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'base_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'base_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'base_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'base_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'end_effector_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'end_effector_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'end_effector_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'end_effector_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_module_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vision_module_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vision_module_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vision_module_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'interface_module_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'interface_module_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'interface_module_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'interface_module_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_laterality)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'arm_laterality)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'arm_laterality)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'arm_laterality)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wrist_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'wrist_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'wrist_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'wrist_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brake_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'brake_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'brake_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'brake_type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CompleteProductConfiguration>) istream)
  "Deserializes a message object of type '<CompleteProductConfiguration>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kin) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'kin) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'model)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'model)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'model)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'model)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'country_code) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'assembly_plant) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'assembly_plant) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_year) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model_year) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'degree_of_freedom)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'degree_of_freedom)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'degree_of_freedom)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'degree_of_freedom)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'base_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'base_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'base_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'base_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'end_effector_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'end_effector_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'end_effector_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'end_effector_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_module_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vision_module_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'vision_module_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'vision_module_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'interface_module_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'interface_module_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'interface_module_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'interface_module_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'arm_laterality)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'arm_laterality)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'arm_laterality)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'arm_laterality)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wrist_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'wrist_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'wrist_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'wrist_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brake_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'brake_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'brake_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'brake_type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CompleteProductConfiguration>)))
  "Returns string type for a message object of type '<CompleteProductConfiguration>"
  "kortex_driver/CompleteProductConfiguration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CompleteProductConfiguration)))
  "Returns string type for a message object of type 'CompleteProductConfiguration"
  "kortex_driver/CompleteProductConfiguration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CompleteProductConfiguration>)))
  "Returns md5sum for a message object of type '<CompleteProductConfiguration>"
  "b73bc25bff314a65c4c98c0970043ccf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CompleteProductConfiguration)))
  "Returns md5sum for a message object of type 'CompleteProductConfiguration"
  "b73bc25bff314a65c4c98c0970043ccf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CompleteProductConfiguration>)))
  "Returns full string definition for message of type '<CompleteProductConfiguration>"
  (cl:format cl:nil "~%string kin~%uint32 model~%CountryCode country_code~%string assembly_plant~%string model_year~%uint32 degree_of_freedom~%uint32 base_type~%uint32 end_effector_type~%uint32 vision_module_type~%uint32 interface_module_type~%uint32 arm_laterality~%uint32 wrist_type~%uint32 brake_type~%================================================================================~%MSG: kortex_driver/CountryCode~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CompleteProductConfiguration)))
  "Returns full string definition for message of type 'CompleteProductConfiguration"
  (cl:format cl:nil "~%string kin~%uint32 model~%CountryCode country_code~%string assembly_plant~%string model_year~%uint32 degree_of_freedom~%uint32 base_type~%uint32 end_effector_type~%uint32 vision_module_type~%uint32 interface_module_type~%uint32 arm_laterality~%uint32 wrist_type~%uint32 brake_type~%================================================================================~%MSG: kortex_driver/CountryCode~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CompleteProductConfiguration>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'kin))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'country_code))
     4 (cl:length (cl:slot-value msg 'assembly_plant))
     4 (cl:length (cl:slot-value msg 'model_year))
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CompleteProductConfiguration>))
  "Converts a ROS message object to a list"
  (cl:list 'CompleteProductConfiguration
    (cl:cons ':kin (kin msg))
    (cl:cons ':model (model msg))
    (cl:cons ':country_code (country_code msg))
    (cl:cons ':assembly_plant (assembly_plant msg))
    (cl:cons ':model_year (model_year msg))
    (cl:cons ':degree_of_freedom (degree_of_freedom msg))
    (cl:cons ':base_type (base_type msg))
    (cl:cons ':end_effector_type (end_effector_type msg))
    (cl:cons ':vision_module_type (vision_module_type msg))
    (cl:cons ':interface_module_type (interface_module_type msg))
    (cl:cons ':arm_laterality (arm_laterality msg))
    (cl:cons ':wrist_type (wrist_type msg))
    (cl:cons ':brake_type (brake_type msg))
))
