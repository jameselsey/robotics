"""Session shutdown joins audio tasks and does not leak delayed probes."""

import asyncio
import threading
from types import SimpleNamespace

import pytest
from senses.conversation_session import run_agent_io, run_session


class Logger:
    def __getattr__(self, _):
        return lambda *_: None


class Agent:
    def __init__(self):
        self.stopped = False

    async def start(self):
        pass

    async def stop(self):
        self.stopped = True

    async def send(self, event):
        pass

    async def receive(self):
        yield {
            "type": "bidi_transcript_stream",
            "text": "hello",
            "role": "assistant",
            "is_final": True,
        }


class Audio:
    def __init__(self, failure=None):
        self.failure = failure
        self.stopped = False

    async def start(self, agent):
        if self.failure == "start":
            raise RuntimeError("audio start failed")

    async def stop(self):
        self.stopped = True

    async def __call__(self, *args):
        if self.failure == "output":
            raise RuntimeError("audio output failed")
        await asyncio.sleep(10)


@pytest.mark.parametrize("failure", [None, "start", "output"])
def test_io_stops_after_receive_completion_or_failure(failure):
    async def scenario():
        agent, mic, speaker = Agent(), Audio(), Audio(failure)
        transcripts = []
        operation = run_agent_io(
            agent, [mic], [speaker], logger=Logger(), publish_transcript=transcripts.append
        )
        if failure:
            with pytest.raises(RuntimeError):
                await asyncio.wait_for(operation, 0.5)
        else:
            # A nonblocking speaker allows the receive iterator to finish.
            async def output(event):
                pass

            await asyncio.wait_for(
                run_agent_io(
                    agent, [mic], [output], logger=Logger(), publish_transcript=transcripts.append
                ),
                0.5,
            )
            operation.close()
        assert agent.stopped and mic.stopped
        if failure:
            assert speaker.stopped
        if failure != "start":
            assert transcripts == [{"role": "assistant", "text": "hello", "is_final": True}]

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "trigger,reason",
    [
        ("maximum", "max session duration"),
        ("idle", "idle timeout"),
        ("stop", "stop requested"),
        ("complete", "agent run completed"),
        ("cancel", None),
    ],
)
def test_session_cleanup(trigger, reason):
    async def scenario():
        stopped = threading.Event()
        finished = set()

        async def task(name):
            try:
                if name == "run" and trigger == "complete":
                    return
                await asyncio.sleep(10)
            finally:
                finished.add(name)

        session = asyncio.create_task(
            run_session(
                task("run"),
                stop_event=stopped,
                activity=SimpleNamespace(idle_seconds=lambda: 100 if trigger == "idle" else 0),
                idle_timeout_seconds=1,
                max_session_seconds=0.04 if trigger == "maximum" else 10,
                heartbeat=task("heartbeat"),
                probe=task("probe"),
                logger=Logger(),
                poll_interval=0.01,
            )
        )
        await asyncio.sleep(0.02)
        if trigger == "stop":
            stopped.set()
        if trigger == "cancel":
            session.cancel()
            with pytest.raises(asyncio.CancelledError):
                await session
        else:
            assert await asyncio.wait_for(session, 0.5) == reason
        assert stopped.is_set()
        assert finished == {"run", "heartbeat", "probe"}

    asyncio.run(scenario())


@pytest.mark.parametrize("trigger", ["input_failure", "cancel"])
def test_io_joins_tasks_before_audio_cleanup(trigger):
    async def scenario():
        receiving = asyncio.Event()
        input_finished = asyncio.Event()

        class StreamingAgent(Agent):
            async def receive(self):
                receiving.set()
                await asyncio.sleep(10)
                yield {}

        class Microphone(Audio):
            async def __call__(self):
                try:
                    await receiving.wait()
                    if trigger == "input_failure":
                        raise RuntimeError("microphone disconnected")
                    await asyncio.sleep(10)
                finally:
                    input_finished.set()

            async def stop(self):
                assert input_finished.is_set()
                await super().stop()

        agent, mic, speaker = StreamingAgent(), Microphone(), Audio()
        operation = asyncio.create_task(
            run_agent_io(
                agent,
                [mic],
                [speaker],
                logger=Logger(),
                publish_transcript=lambda _: None,
            )
        )
        await receiving.wait()
        if trigger == "cancel":
            operation.cancel()
            with pytest.raises(asyncio.CancelledError):
                await operation
        else:
            with pytest.raises(RuntimeError, match="microphone disconnected"):
                await asyncio.wait_for(operation, 0.5)
        assert agent.stopped and mic.stopped and speaker.stopped

    asyncio.run(scenario())
