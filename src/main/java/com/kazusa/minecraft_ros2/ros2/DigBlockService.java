package com.kazusa.minecraft_ros2.ros2;

import java.time.Instant;
import java.time.Duration;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import net.minecraft.server.MinecraftServer;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.BlockHitResult;
import net.minecraft.world.phys.HitResult;
import net.minecraft.core.BlockPos;
import net.minecraftforge.server.ServerLifecycleHooks;
import net.minecraft.network.protocol.game.ClientboundBlockDestructionPacket;

import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.service.Service;
import org.ros2.rcljava.service.RMWRequestId;

import minecraft_msgs.srv.DigBlock; 
import minecraft_msgs.srv.DigBlock_Request;
import minecraft_msgs.srv.DigBlock_Response;
import std_msgs.msg.Float32;

public class DigBlockService extends BaseComposableNode {

    // 掘削結果を保持するためのクラス
    // ラムダ内で最終値を書き換えられるように volatile を使う
    private static class ResultHolder {
        volatile int code = 1;
    }

    private final Service<DigBlock> srvServer;
    private final Publisher<Float32> progressPublisher;

    public DigBlockService() {
        super("dig_block_service");

        try {
            this.srvServer = this.node.<DigBlock>createService(
                DigBlock.class,
                "dig_block",
                (RMWRequestId header, DigBlock_Request request,
                    DigBlock_Response response)
                    -> this.handleService(header, request, response)
            );
            this.progressPublisher = this.node.<Float32>createPublisher(
                Float32.class,
                "dig_block/progress"
            );
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException("サービス生成に失敗", e);
        }
    }

    private void handleService(final RMWRequestId header, DigBlock_Request request, DigBlock_Response response) {
        float timeoutSec = request.getTimeout();

        MinecraftServer server = ServerLifecycleHooks.getCurrentServer();
        if (server == null) {
            response.setResultCode(1);
            return;
        }

        // サーバースレッド上の掘削タスク完了を待つためのカウントダウンラッチ
        CountDownLatch latch = new CountDownLatch(1);

        // 掘削結果を保持する変数（ラムダ内で最終値を書き換えられるよう AtomicReference や配列でも可）
        final ResultHolder resultHolder = new ResultHolder();

        server.execute(() -> {
            // プレイヤー取得
            if (server.getPlayerList().getPlayers().isEmpty()) {
                resultHolder.code = 1; // ブロックなし扱い
                latch.countDown();
                return;
            }

            ServerPlayer player = server.getPlayerList().getPlayers().get(0);
            Level lvl = player.level();
            if (!(lvl instanceof ServerLevel)) {
                resultHolder.code = 1; // ブロックなし扱い
                latch.countDown();
                return;
            }
            ServerLevel world = (ServerLevel) lvl;

            // レイキャストで掘るブロックを探す
            double reachDistance = 5.0D;
            HitResult hit = player.pick(reachDistance, 0.0F, false);
            if (hit.getType() != HitResult.Type.BLOCK || !(hit instanceof BlockHitResult)) {
                resultHolder.code = 1;
                latch.countDown();
                return;
            }
            BlockHitResult blockHit = (BlockHitResult) hit;
            BlockPos targetPos = blockHit.getBlockPos();

            // 硬度・ツール速度から所要時間を計算
            float hardness = world.getBlockState(targetPos).getDestroySpeed(world, targetPos);
            if (hardness < 0.0f) {
                resultHolder.code = 1; // 破壊出来ないブロックしかない場合
                latch.countDown();
                return;
            }
            float toolSpeed = player.getMainHandItem().getDestroySpeed(world.getBlockState(targetPos));
            if (toolSpeed <= 1.0f) toolSpeed = 1.0f;

            float totalNeededDamage = hardness * 30.0f;
            float ticksNeeded = totalNeededDamage / toolSpeed;
            double secondsNeeded = ticksNeeded / 20.0;
            boolean noTimeout = (timeoutSec <= 0);

            Instant startTime = Instant.now();

            // 掘削ループ：0.1秒ごとにひび割れとフィードバックを送る
            while (true) {
                double elapsedSec = Duration.between(startTime, Instant.now()).toMillis() / 1000.0;

                // タイムアウト判定
                if (!noTimeout && elapsedSec >= timeoutSec) {
                    // タイムアウト → ひび割れを消す
                    ClientboundBlockDestructionPacket clearPacket =
                        new ClientboundBlockDestructionPacket(player.getId(), targetPos, -1);
                    for (ServerPlayer p : server.getPlayerList().getPlayers()) {
                        p.connection.send(clearPacket);
                    }

                    resultHolder.code = 2;
                    latch.countDown();
                    return;
                }

                // 掘削完了判定
                if (elapsedSec >= secondsNeeded) {
                    // 終了直前に「ステージ9」のひび割れを送る（任意）
                    int finalStage = 9;
                    ClientboundBlockDestructionPacket finalPacket =
                        new ClientboundBlockDestructionPacket(player.getId(), targetPos, finalStage);
                    for (ServerPlayer p : server.getPlayerList().getPlayers()) {
                        p.connection.send(finalPacket);
                    }
                    // 少し待ってクライアントに描画させる
                    try { Thread.sleep(50); } catch (InterruptedException ignored) {}

                    // プレイヤーのアクションとしてブロック破壊
                    player.gameMode.destroyBlock(targetPos);

                    Float32 fb = new Float32();
                    fb.setData(1.0f); // 完了時の進捗は 1.0
                    this.progressPublisher.publish(fb);

                    // 破壊後にひび割れを消す
                    ClientboundBlockDestructionPacket clearAfterDestroy =
                        new ClientboundBlockDestructionPacket(player.getId(), targetPos, -1);
                    for (ServerPlayer p : server.getPlayerList().getPlayers()) {
                        p.connection.send(clearAfterDestroy);
                    }

                    resultHolder.code = 0; // 成功
                    latch.countDown();
                    return;
                }

                // 進捗を 0~9 にマッピングしてひび割れを送信
                float progressRatio = (float)(elapsedSec / secondsNeeded);
                if (progressRatio > 1.0f) progressRatio = 1.0f;
                int breakStage = (int)(progressRatio * 9.0f);
                if (breakStage < 0) breakStage = 0;
                if (breakStage > 9) breakStage = 9;

                ClientboundBlockDestructionPacket crackPacket =
                    new ClientboundBlockDestructionPacket(player.getId(), targetPos, breakStage);
                for (ServerPlayer p : server.getPlayerList().getPlayers()) {
                    p.connection.send(crackPacket);
                }

                // ROS へのフィードバック publish
                Float32 fb = new Float32();
                fb.setData(progressRatio);
                this.progressPublisher.publish(fb);

                // 0.1秒待機
                try {
                    Thread.sleep(100);
                } catch (InterruptedException ex) {
                    // 割り込みが来たら「キャンセル扱い」としてひび割れを消す
                    ClientboundBlockDestructionPacket clearOnCancel =
                        new ClientboundBlockDestructionPacket(player.getId(), targetPos, -1);
                    for (ServerPlayer p : server.getPlayerList().getPlayers()) {
                        p.connection.send(clearOnCancel);
                    }
                    resultHolder.code = 3; // 割り込みによるキャンセル
                    latch.countDown();
                    return;
                }
            }
        });

        // ここで「掘削完了（またはタイムアウト）ラッチが開くまで」最大 (timeoutSec + 1) 秒だけブロックする
        try {
            // timeoutSec が 0 以下の場合は「ずっと待つ」ように Long.MAX_VALUE を使う例
            if (timeoutSec <= 0) {
                latch.await();
            } else {
                // タイムアウト＋余裕１秒分待つ
                boolean finishedInTime = latch.await((long)(timeoutSec + 1), TimeUnit.SECONDS);
                if (!finishedInTime) {
                    // 何らかの理由でラッチが開かなかった場合は強制タイムアウト
                    resultHolder.code = 2;
                }
            }
        } catch (InterruptedException ex) {
            // こちらも割り込まれたらキャンセル扱い
            resultHolder.code = 3;
        }

        // サーバースレッドで設定された resultHolder.code を使ってレスポンスを返却
        response.setResultCode(resultHolder.code);
    }

    public void shutdown() {
        this.node.dispose();
    }
}
