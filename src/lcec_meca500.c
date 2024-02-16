#include "lcec.h"
#include "lcec_meca500.h"

typedef struct {
	
  // robot ctrl
  hal_bit_t *deactivate;
  hal_bit_t *activate;
  hal_bit_t *home;
  hal_bit_t *reset_error;
  hal_bit_t *sim_mode;
  
  // motion ctrl
  hal_u32_t *move_id;
  hal_bit_t *set_point;
  hal_bit_t *pause;
  hal_bit_t *clear_move;
  hal_bit_t *reset_pstop;
  
  // move
  hal_u32_t   *move_cmd;
  hal_float_t *move_a1;
  hal_float_t *move_a2;
  hal_float_t *move_a3;
  hal_float_t *move_a4;
  hal_float_t *move_a5;
  hal_float_t *move_a6;
  
    // robot status
  hal_bit_t *stat_busy;
  hal_bit_t *stat_activated;
  hal_bit_t *stat_homed;
  hal_bit_t *stat_sim_activated;
  hal_u32_t *stat_error_code;
  
  // motion status
  hal_u32_t *mstat_check_point;
  hal_u32_t *mstat_move_id;
  hal_u32_t *mstat_fifo_space;
  hal_bit_t *mstat_paused;
  hal_bit_t *mstat_eob;
  hal_bit_t *mstat_eom;
  hal_bit_t *mstat_cleared;
  hal_bit_t *mstat_pstop;
  hal_bit_t *mstat_ex_torque;
  
  // encoders
  hal_float_t *enc_a1;
  hal_float_t *enc_a2;
  hal_float_t *enc_a3;
  hal_float_t *enc_a4;
  hal_float_t *enc_a5;
  hal_float_t *enc_a6;

  // end effector pose
  hal_float_t *pose_a1;
  hal_float_t *pose_a2;
  hal_float_t *pose_a3;
  hal_float_t *pose_a4;
  hal_float_t *pose_a5;
  hal_float_t *pose_a6;
  
  // joint velocity
  hal_float_t *vel_a1;
  hal_float_t *vel_a2;
  hal_float_t *vel_a3;
  hal_float_t *vel_a4;
  hal_float_t *vel_a5;
  hal_float_t *vel_a6;

  // torque
  hal_float_t *torque_a1;
  hal_float_t *torque_a2;
  hal_float_t *torque_a3;
  hal_float_t *torque_a4;
  hal_float_t *torque_a5;
  hal_float_t *torque_a6;
  
  // accelerometer
  hal_s32_t *accel_x;
  hal_s32_t *accel_y;
  hal_s32_t *accel_z;
  
  unsigned int deactivate_pdo_os, activate_pdo_os, home_pdo_os, reset_error_pdo_os, sim_mode_pdo_os, move_id_pdo_os;
  unsigned int deactivate_pdo_bp, activate_pdo_bp, home_pdo_bp, reset_error_pdo_bp, sim_mode_pdo_bp, move_id_pdo_bp;
  unsigned int set_point_pdo_os, pause_pdo_os, clear_move_pdo_os, reset_pstop_pdo_os;
  unsigned int set_point_pdo_bp, pause_pdo_bp, clear_move_pdo_bp, reset_pstop_pdo_bp;
  unsigned int move_cmd_pdo_os, move_a1_pdo_os, move_a2_pdo_os, move_a3_pdo_os, move_a4_pdo_os, move_a5_pdo_os, move_a6_pdo_os;
  unsigned int move_cmd_pdo_bp, move_a1_pdo_bp, move_a2_pdo_bp, move_a3_pdo_bp, move_a4_pdo_bp, move_a5_pdo_bp, move_a6_pdo_bp;
  unsigned int stat_busy_pdo_os, stat_activated_pdo_os, stat_homed_pdo_os, stat_sim_activated_pdo_os, stat_error_code_pdo_os;
  unsigned int stat_busy_pdo_bp, stat_activated_pdo_bp, stat_homed_pdo_bp, stat_sim_activated_pdo_bp, stat_error_code_pdo_bp; 
  unsigned int mstat_check_point_pdo_os, mstat_move_id_pdo_os, mstat_fifo_space_pdo_os;
  unsigned int mstat_check_point_pdo_bp, mstat_move_id_pdo_bp, mstat_fifo_space_pdo_bp; 
  unsigned int mstat_paused_pdo_os, mstat_eob_pdo_os, mstat_eom_pdo_os;
  unsigned int mstat_paused_pdo_bp, mstat_eob_pdo_bp, mstat_eom_pdo_bp; 
  unsigned int mstat_cleared_pdo_os, mstat_pstop_pdo_os, mstat_ex_torque_pdo_os;
  unsigned int mstat_cleared_pdo_bp, mstat_pstop_pdo_bp, mstat_ex_torque_pdo_bp;
  unsigned int enc_a1_pdo_os, enc_a2_pdo_os, enc_a3_pdo_os, enc_a4_pdo_os, enc_a5_pdo_os, enc_a6_pdo_os;
  unsigned int enc_a1_pdo_bp, enc_a2_pdo_bp, enc_a3_pdo_bp, enc_a4_pdo_bp, enc_a5_pdo_bp, enc_a6_pdo_bp;
  unsigned int pose_a1_pdo_os, pose_a2_pdo_os, pose_a3_pdo_os, pose_a4_pdo_os, pose_a5_pdo_os, pose_a6_pdo_os;
  unsigned int pose_a1_pdo_bp, pose_a2_pdo_bp, pose_a3_pdo_bp, pose_a4_pdo_bp, pose_a5_pdo_bp, pose_a6_pdo_bp;
  unsigned int vel_a1_pdo_os, vel_a2_pdo_os, vel_a3_pdo_os, vel_a4_pdo_os, vel_a5_pdo_os, vel_a6_pdo_os;
  unsigned int vel_a1_pdo_bp, vel_a2_pdo_bp, vel_a3_pdo_bp, vel_a4_pdo_bp, vel_a5_pdo_bp, vel_a6_pdo_bp; 
  unsigned int torque_a1_pdo_os, torque_a2_pdo_os, torque_a3_pdo_os, torque_a4_pdo_os, torque_a5_pdo_os, torque_a6_pdo_os;
  unsigned int torque_a1_pdo_bp, torque_a2_pdo_bp, torque_a3_pdo_bp, torque_a4_pdo_bp, torque_a5_pdo_bp, torque_a6_pdo_bp;
  unsigned int accel_x_pdo_os, accel_y_pdo_os, accel_z_pdo_os;
  unsigned int accel_x_pdo_bp, accel_y_pdo_bp, accel_z_pdo_bp;
  
} lcec_meca500_data_t;


static const lcec_pindesc_t slave_pins[] = {
	
  // robot ctrl
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, deactivate), "%s.%s.%s.deactivate" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, activate),   "%s.%s.%s.activate" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, home),       "%s.%s.%s.home" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, reset_error),"%s.%s.%s.reset" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, sim_mode),   "%s.%s.%s.sim-mode" },
  
    // motion ctrl
  { HAL_U32,   HAL_IO, offsetof(lcec_meca500_data_t, move_id),    "%s.%s.%s.move-id" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, set_point),  "%s.%s.%s.set-point" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, pause),      "%s.%s.%s.pause" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, clear_move), "%s.%s.%s.clear-move" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, reset_pstop),"%s.%s.%s.reset-pstop" },
  
  // move command
  { HAL_U32,   HAL_IO, offsetof(lcec_meca500_data_t, move_cmd),"%s.%s.%s.move-cmd" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, move_a1), "%s.%s.%s.move-a1" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, move_a2), "%s.%s.%s.move-a2" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, move_a3), "%s.%s.%s.move-a3" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, move_a4), "%s.%s.%s.move-a4" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, move_a5), "%s.%s.%s.move-a5" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, move_a6), "%s.%s.%s.move-a6" },
  
    // robot status
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, stat_busy),         "%s.%s.%s.stat-busy" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, stat_activated),    "%s.%s.%s.stat-activated" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, stat_homed),        "%s.%s.%s.stat-homed" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, stat_sim_activated),"%s.%s.%s.stat-sim-activated" },
  { HAL_U32,   HAL_IO, offsetof(lcec_meca500_data_t, stat_error_code),   "%s.%s.%s.stat-error-code" },
  
  // motion status
  { HAL_U32,   HAL_IO, offsetof(lcec_meca500_data_t, mstat_check_point),   "%s.%s.%s.mstat-check-point" },
  { HAL_U32,   HAL_IO, offsetof(lcec_meca500_data_t, mstat_move_id),       "%s.%s.%s.mstat-status-move-id" },
  { HAL_U32,   HAL_IO, offsetof(lcec_meca500_data_t, mstat_fifo_space),    "%s.%s.%s.mstat-fifo-space" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, mstat_paused),        "%s.%s.%s.mstat-paused" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, mstat_eob),           "%s.%s.%s.mstat-eob" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, mstat_eom),           "%s.%s.%s.mstat-eon" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, mstat_cleared),       "%s.%s.%s.mstat-cleared" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, mstat_pstop),         "%s.%s.%s.mstat-pstop" },
  { HAL_BIT,   HAL_IO, offsetof(lcec_meca500_data_t, mstat_ex_torque),     "%s.%s.%s.mstat-excessive-torque" },
  
  // joint encoders
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, enc_a1), "%s.%s.%s.enc-a1" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, enc_a2), "%s.%s.%s.enc-a2" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, enc_a3), "%s.%s.%s.enc-a3" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, enc_a4), "%s.%s.%s.enc-a4" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, enc_a5), "%s.%s.%s.enc-a5" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, enc_a6), "%s.%s.%s.enc-a6" },
  
  // end effector pose
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, pose_a1), "%s.%s.%s.pose-a1" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, pose_a2), "%s.%s.%s.pose-a2" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, pose_a3), "%s.%s.%s.pose-a3" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, pose_a4), "%s.%s.%s.pose-a4" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, pose_a5), "%s.%s.%s.pose-a5" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, pose_a6), "%s.%s.%s.pose-a6" },
  
  // joint velocity
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, vel_a1), "%s.%s.%s.vel-a1" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, vel_a2), "%s.%s.%s.vel-a2" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, vel_a3), "%s.%s.%s.vel-a3" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, vel_a4), "%s.%s.%s.vel-a4" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, vel_a5), "%s.%s.%s.vel-a5" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, vel_a6), "%s.%s.%s.vel-a6" },
  
  // joint torque
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, torque_a1), "%s.%s.%s.torque-a1" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, torque_a2), "%s.%s.%s.torque-a2" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, torque_a3), "%s.%s.%s.torque-a3" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, torque_a4), "%s.%s.%s.torque-a4" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, torque_a5), "%s.%s.%s.torque-a5" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_meca500_data_t, torque_a6), "%s.%s.%s.torque-a6" },
  
  // accelerometer
  { HAL_S32,   HAL_IO, offsetof(lcec_meca500_data_t, accel_x), "%s.%s.%s.accel-x" },
  { HAL_S32,   HAL_IO, offsetof(lcec_meca500_data_t, accel_y), "%s.%s.%s.accel-y" },
  { HAL_S32,   HAL_IO, offsetof(lcec_meca500_data_t, accel_z), "%s.%s.%s.accel-z" },
  
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
  
};

static ec_pdo_entry_info_t lcec_meca500_pdo_out_ctrl[] = {
    {0x7200, 0x01, 1},  /* Deactivate */
    {0x7200, 0x02, 1},  /* Activate */
    {0x7200, 0x03, 1},  /* Home */
    {0x7200, 0x04, 1},  /* Reset Error */
    {0x7200, 0x05, 1},  /* Sim Mode */
    {0x0000, 0x00, 27}  /* Gap */
};

static ec_pdo_entry_info_t lcec_meca500_pdo_out_motion[] = {
    {0x7310, 0x01, 16}, /* Move ID */
    {0x7310, 0x02, 1},  /* SetPoint */
    {0x7310, 0x03, 1},  /* Pause */
    {0x7310, 0x04, 1},  /* Clear Move */
    {0x7310, 0x05, 1},  /* Reset PStop */
    {0x0000, 0x00, 12}  /* Gap */
};  

static ec_pdo_entry_info_t lcec_meca500_pdo_out_move[] = {
    {0x7305, 0x00, 32}, /* Move Command */
    {0x7306, 0x01, 32}, /* SubIndex 001 */
    {0x7306, 0x02, 32}, /* SubIndex 002 */
    {0x7306, 0x03, 32}, /* SubIndex 003 */
    {0x7306, 0x04, 32}, /* SubIndex 004 */
    {0x7306, 0x05, 32}, /* SubIndex 005 */
    {0x7306, 0x06, 32}  /* SubIndex 006 */
};

ec_pdo_info_t lcec_meca500_pdos_out[] = {
    {0x1600, 6,  lcec_meca500_pdo_out_ctrl},    /* Robot Control */
    {0x1601, 6,  lcec_meca500_pdo_out_motion},  /* Motion Control */
    {0x1602, 7,  lcec_meca500_pdo_out_move}     /* Move */
};

static ec_sync_info_t lcec_meca500_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL,                  EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, NULL,                  EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 3, lcec_meca500_pdos_out, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  0, NULL,                  EC_WD_DISABLE},
    {0xff}
};

// callbacks
void lcec_meca500_read(struct lcec_slave *slave, long period);
void lcec_meca500_write(struct lcec_slave *slave, long period);

int lcec_meca500_init(int comp_id, struct lcec_slave *s, ec_pdo_entry_reg_t *r) {
  lcec_master_t *m = s->master;
  lcec_meca500_data_t *hd;
  int err;
  
  // init callbacks
  s->proc_read  = lcec_meca500_read;
  s->proc_write = lcec_meca500_write;
  
  // alloc hal memory
  if ((hd = hal_malloc(sizeof(lcec_meca500_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", m->name, s->name);
    return -EIO;
  }
  memset(hd, 0, sizeof(lcec_meca500_data_t));
  s->hal_data = hd;
  
  // init sync info
  s->sync_info = lcec_meca500_syncs;
  
  // init robot ctrl
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7200, 0x01,   &hd->deactivate_pdo_os,    &hd->deactivate_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7200, 0x02,   &hd->activate_pdo_os,      &hd->activate_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7200, 0x03,   &hd->home_pdo_os,          &hd->home_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7200, 0x04,   &hd->reset_error_pdo_os,   &hd->reset_error_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7200, 0x05,   &hd->sim_mode_pdo_os,      &hd->sim_mode_pdo_bp);
  //ecrt_slave_config_pdo_mapping_add(0, 0x1600, 0x0000, 0x00, 27);
  
  // init robot motion control
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7310, 0x01,   &hd->move_id_pdo_os,       &hd->move_id_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7310, 0x02,   &hd->set_point_pdo_os,     &hd->set_point_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7310, 0x03,   &hd->pause_pdo_os,         &hd->pause_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7310, 0x04,   &hd->clear_move_pdo_os,    &hd->clear_move_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7310, 0x05,   &hd->reset_pstop_pdo_os,   &hd->reset_pstop_pdo_bp);
  //ecrt_slave_config_pdo_mapping_add(0, 0x1600, 0x0000, 0x00, 12);
  
  // init robot move command
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7305, 0x00,   &hd->move_cmd_pdo_os,      &hd->move_cmd_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7306, 0x01,   &hd->move_a1_pdo_os,       &hd->move_a1_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7306, 0x02,   &hd->move_a2_pdo_os,       &hd->move_a2_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7306, 0x03,   &hd->move_a3_pdo_os,       &hd->move_a3_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7306, 0x04,   &hd->move_a4_pdo_os,       &hd->move_a4_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7306, 0x05,   &hd->move_a5_pdo_os,       &hd->move_a5_pdo_bp);
  LCEC_PDO_INIT(r, s->index, s->vid, s->pid, 0x7306, 0x06,   &hd->move_a6_pdo_os,       &hd->move_a6_pdo_bp);
  
  // export pins
  if ((err = lcec_pin_newf_list(hd, slave_pins, LCEC_MODULE_NAME, m->name, s->name)) != 0) {
    return err;
  }
  
  return 0;
}


void lcec_meca500_read(struct lcec_slave *slave, long period){
  lcec_master_t *master = slave->master;
  uint8_t *pd = master->process_data;
  lcec_meca500_data_t *hal_data = (lcec_meca500_data_t *) slave->hal_data;

}


void lcec_meca500_write(struct lcec_slave *slave, long period){
  lcec_master_t *master = slave->master;
  uint8_t *pd = master->process_data;
  lcec_meca500_data_t *hal_data = (lcec_meca500_data_t *) slave->hal_data;

}
