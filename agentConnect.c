
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <mapper/mapper.h>

struct _agentInfo
{
    char *influence_device_name;
    char *xagora_device_name;

    int linked_influence;
    int connected;

    mapper_admin admin;
    mapper_device dev;
    mapper_monitor mon;
    mapper_db db;
} agentInfo;

mapper_signal sig_pos_in,
              sig_pos_out,
              sig_vel_in,
              sig_vel_out,
              sig_accel_in,
              sig_accel_out,
              sig_force;
mapper_timetag_t tt;

float mass = 1.0;
float gain = 0.0001;
float damping = 0.6;
float limit = 0.1;

#define WIDTH  500
#define HEIGHT 500

int numInstances = 1;
int done = 0;

void make_influence_connections()
{
    char signame1[1024], signame2[1024];
    struct _agentInfo *info = &agentInfo;

    sprintf(signame1, "%s/node/observation", info->influence_device_name);
    sprintf(signame2, "%s/force", mdev_name(info->dev));
    mapper_monitor_connect(info->mon, signame1, signame2, 0, 0);

    sprintf(signame1, "%s/position", mdev_name(info->dev));
    sprintf(signame2, "%s/node/position", info->influence_device_name);
    mapper_monitor_connect(info->mon, signame1, signame2, 0, 0);
}

void make_xagora_connections()
{
    char signame1[1024], signame2[1024];
    struct _agentInfo *info = &agentInfo;

    sprintf(signame1, "%s/position", mdev_name(info->dev));

    sprintf(signame2, "%s/Butterfly%d",
            info->xagora_device_name, mdev_ordinal(info->dev));

    mapper_monitor_connect(info->mon, signame1, signame2, 0, 0);
}

void init_instance(int id)
{
    float init[] = {0, 0};

    // initialize accelerations to zero
    msig_update_instance(sig_accel_in, id, &init, 1, MAPPER_NOW);

    // initialize velocities to zero
    msig_update_instance(sig_vel_in, id, &init, 1, MAPPER_NOW);

    // random position
    init[0] = rand()%1000*0.002-1.0;
    init[1] = rand()%1000*0.002-1.0;
    msig_update_instance(sig_pos_in, id, &init, 1, MAPPER_NOW);
}

void release_instance(int instance_id)
{
    msig_release_instance(sig_pos_in, instance_id, MAPPER_NOW);
    msig_release_instance(sig_pos_out, instance_id, MAPPER_NOW);
    msig_release_instance(sig_vel_in, instance_id, MAPPER_NOW);
    msig_release_instance(sig_vel_out, instance_id, MAPPER_NOW);
    msig_release_instance(sig_accel_in, instance_id, MAPPER_NOW);
    msig_release_instance(sig_accel_out, instance_id, MAPPER_NOW);
    msig_release_instance(sig_force, instance_id, MAPPER_NOW);
}

void force_handler(mapper_signal msig,
                   mapper_db_signal props,
                   int instance_id,
                   void *value,
                   int count,
                   mapper_timetag_t *timetag)
{
    if (!value) {
        // release all assocaiated signal instances
        release_instance(instance_id);
        // reinit with random values
        init_instance(instance_id);
        return;
    }
    float *force = (float *)value;
    mapper_signal sig = (mapper_signal)props->user_data;
    float accel[2], *paccel;

    paccel = (float *)msig_instance_value(sig, instance_id, 0);
    if (!paccel) {
        printf("error: force_handler cannot retrieve acceleration for instance %i\n",
               instance_id);
        return;
    }

    accel[0] = paccel[0] + force[0] / mass * gain;
    accel[1] = paccel[1] + force[1] / mass * gain;
    msig_update_instance(sig, instance_id, &accel, 1, MAPPER_NOW);
}

void dev_db_callback(mapper_db_device record,
                     mapper_db_action_t action,
                     void *user)
{
    // if we see /influence.1 or /XAgora.1, send /link message
    struct _agentInfo *info = (struct _agentInfo*)user;

    if (action == MDB_NEW) {
        if (strcmp(record->name, info->influence_device_name)==0) {
            mapper_db_link_t props;
            props.num_scopes = 1;
            char *name = (char *)mdev_name(info->dev);
            props.scope_names = &name;
            mapper_monitor_link(info->mon, mdev_name(info->dev), record->name, 0, 0);
            mapper_monitor_link(info->mon, record->name, mdev_name(info->dev),
                                &props, LINK_NUM_SCOPES | LINK_SCOPE_NAMES);
        }
        else if (strcmp(record->name, info->xagora_device_name)==0) {
            // make links to XAgora
        }
    }
    else if (action == MDB_REMOVE) {
        if (strcmp(record->name, info->influence_device_name)==0) {
            mapper_monitor_unlink(info->mon, mdev_name(info->dev),
                                  record->name);
            info->linked_influence = 0;
        }
    }
}

void link_db_callback(mapper_db_link record,
                      mapper_db_action_t action,
                      void *user)
{
    // if we see our links, send /connect messages
    struct _agentInfo *info = (struct _agentInfo*)user;

    if (action == MDB_NEW) {
        if (((strcmp(record->dest_name, info->influence_device_name)==0) &&
            (strcmp(record->src_name, mdev_name(info->dev))==0)))
            info->linked_influence |= 0x01;
        else if (((strcmp(record->src_name, info->influence_device_name)==0) &&
             (strcmp(record->dest_name, mdev_name(info->dev))==0))) {
            info->linked_influence |= 0x02;
            if (info->linked_influence & 0x03)
                make_influence_connections();
        }
        else if ((strcmp(record->src_name, mdev_name(info->dev))==0) &&
              (strcmp(record->dest_name, info->xagora_device_name)==0)) {
            make_xagora_connections();
        }
    }
    else if (action == MDB_REMOVE) {
        if ((strcmp(record->src_name, info->influence_device_name)==0) ||
            (strcmp(record->dest_name, info->influence_device_name)==0))
            info->linked_influence--;
    }
}

struct _agentInfo *agentInit()
{
    int i;
    struct _agentInfo *info = &agentInfo;
    memset(info, 0, sizeof(struct _agentInfo));

    info->influence_device_name = strdup("/influence.1");
    info->xagora_device_name = strdup("/XAgora_receiver.1");

    info->admin = mapper_admin_new(0, 0, 0);

    // add device
    info->dev = mdev_new("agent", 0, info->admin);
    while (!mdev_ready(info->dev)) {
        mdev_poll(info->dev, 100);
    }
    printf("ordinal: %d\n", mdev_ordinal(info->dev));
    fflush(stdout);

    // add monitor and monitor callbacks
    info->mon = mapper_monitor_new(info->admin, 0);
    info->db  = mapper_monitor_get_db(info->mon);
    mapper_db_add_device_callback(info->db, dev_db_callback, info);
    mapper_db_add_link_callback(info->db, link_db_callback, info);

    // add signals
    float mn=-1, mx=1;

    // Add acceleration signals
    sig_accel_in = mdev_add_input(info->dev, "acceleration", 2, 'f', 0,
                                  &mn, &mx, 0, 0);
    msig_reserve_instances(sig_accel_in, numInstances-1);
    sig_accel_out = mdev_add_output(info->dev, "acceleration", 2, 'f', 0, &mn, &mx);
    msig_reserve_instances(sig_accel_out, numInstances-1);

    // Add force signals
    sig_force = mdev_add_input(info->dev, "force", 2, 'f', "N", &mn, &mx,
                               force_handler, sig_accel_in);
    msig_reserve_instances(sig_force, numInstances-1);

    // Add velocity signals
    sig_vel_in = mdev_add_input(info->dev, "velocity", 2, 'f', "m/s",
                                &mn, &mx, 0, 0);
    msig_reserve_instances(sig_vel_in, numInstances-1);
    sig_vel_out = mdev_add_output(info->dev, "velocity", 2, 'f', "m/s", &mn, &mx);
    msig_reserve_instances(sig_vel_out, numInstances-1);

    // add position signals
    sig_pos_in = mdev_add_input(info->dev, "position", 2, 'f', 0,
                                &mn, &mx, 0, 0);
    msig_reserve_instances(sig_pos_in, numInstances-1);
    sig_pos_out = mdev_add_output(info->dev, "position", 2, 'f', 0, &mn, &mx);
    msig_reserve_instances(sig_pos_out, numInstances-1);

    // initialize positions to random values
    for (i=0; i<numInstances; i++) {
        init_instance(i);
    }

    return info;
}

void agentLogout()
{
    printf("Cleaning up...\n");
    struct _agentInfo *info = &agentInfo;

    int i;
    mdev_now(info->dev, &tt);
    mdev_start_queue(info->dev, tt);
    for (i=0; i<numInstances; i++) {
        msig_release_instance(sig_pos_out, i, tt);
        msig_release_instance(sig_vel_out, i, tt);
        msig_release_instance(sig_accel_out, i, tt);
    }
    mdev_send_queue(info->dev, tt);

    if (info->influence_device_name) {
        mapper_monitor_unlink(info->mon,
                              info->influence_device_name,
                              mdev_name(info->dev));
        free(info->influence_device_name);
    }
    if (info->xagora_device_name) {
        mapper_monitor_unlink(info->mon,
                              mdev_name(info->dev),
                              info->xagora_device_name);
        free(info->xagora_device_name);
    }
    if (info->dev)
        mdev_free(info->dev);
    if (info->db) {
        mapper_db_remove_device_callback(info->db, dev_db_callback, info);
        mapper_db_remove_link_callback(info->db, link_db_callback, info);
    }
    if (info->mon)
        mapper_monitor_free(info->mon);
    if (info->admin) {
        mapper_admin_free(info->admin);
    }
    memset(info, 0, sizeof(struct _agentInfo));
}

void ctrlc(int sig)
{
    done = 1;
}

int main(int argc, char *argv[])
{
    int i, j;
    if (argc > 1)
        numInstances = atoi(argv[1]);

    signal(SIGINT, ctrlc);

    srand(100);

    struct _agentInfo *info = agentInit();
    if (!info->dev)
        goto done;

    float *paccel, *pvel, *ppos;
    float accel[2], vel[2], pos[2];

    while (!mdev_ready(info->dev)) {
        mapper_monitor_poll(info->mon, 0);
        mdev_poll(info->dev, 10);
    }

    while (!done) {
        mapper_monitor_poll(info->mon, 0);
        mdev_poll(info->dev, 20);

        mdev_now(info->dev, &tt);
        mdev_start_queue(info->dev, tt);
        for (i=0; i<numInstances; i++) {
            paccel = (float *)msig_instance_value(sig_accel_in, i, 0);
            pvel = (float *)msig_instance_value(sig_vel_in, i, 0);
            if (!pvel) {
                printf("couldn't retrieve vel value for instance %i\n", i);
                continue;
            }
            if (!paccel) {
                printf("couldn't retrieve accel value for instance %i\n", i);
                continue;
            }

            ppos = (float *)msig_instance_value(sig_pos_in, i, 0);
            if (!ppos) {
                printf("couldn't retrieve pos value for instance %i\n", i);
                continue;
            }

            for (j = 0; j < 2; j++) {
                accel[j] = paccel[j] * 0.9;
                vel[j] = pvel[j] * 0.9 + accel[j];
                pos[j] = ppos[j] + vel[j];

                if (pos[j] < -1) {
                    pos[j] = -1;
                    vel[j] *= -0.95;
                }
                if (pos[j] >= 1) {
                    pos[j] = 1;
                    vel[j] *= -0.95;
                }

                accel[j] = 0;
            }

            msig_update_instance(sig_accel_in, i, &accel, 1, tt);
            msig_update_instance(sig_accel_out, i, &accel, 1, tt);
            msig_update_instance(sig_vel_in, i, &vel, 1, tt);
            msig_update_instance(sig_vel_out, i, &vel, 1, tt);
            msig_update_instance(sig_pos_in, i, &pos, 1, tt);
            msig_update_instance(sig_pos_out, i, &pos, 1, tt);
        }
        mdev_send_queue(info->dev, tt);
    }

done:
    agentLogout();
    return 0;
}
