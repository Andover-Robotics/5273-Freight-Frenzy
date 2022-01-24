package org.firstinspires.ftc.teamcode.d_util.utilclasses;

public class Task {

    String id;
    Runnable task;

    public Task(String id, Runnable task) {
        this.id = id;
        this.task = task;
    }

    public String getId() {
        return id;
    }

    public Runnable getTask() {
        return task;
    }

}
