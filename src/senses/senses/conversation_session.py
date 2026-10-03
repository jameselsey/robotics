"""Async conversation lifecycle without ROS, hardware, or inference imports."""

import asyncio
import threading


async def run_agent_io(agent, inputs, outputs, *, logger, publish_transcript) -> None:
    stopping_io = threading.Event()

    async def start_io() -> None:
        for io in [*inputs, *outputs]:
            start = getattr(io, "start", None)
            if start is not None:
                await start(agent)

    async def stop_io() -> None:
        for io in [*inputs, *outputs]:
            stop = getattr(io, "stop", None)
            if stop is not None:
                try:
                    await stop()
                except Exception as exc:
                    logger.warn(
                        f"IO stop failed for {type(io).__name__}: {type(exc).__name__}: {exc}"
                    )

    async def run_inputs() -> None:
        try:
            while True:
                for input_ in inputs:
                    event = await input_()
                    await agent.send(event)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            if stopping_io.is_set():
                logger.debug(
                    f"Nova input/send loop stopped during shutdown: {type(exc).__name__}: {exc}"
                )
                return
            logger.error(f"Nova input/send loop failed: {type(exc).__name__}: {exc}")
            raise

    async def run_outputs(inputs_task: asyncio.Task) -> None:
        try:
            async for event in agent.receive():
                event_type = (
                    event.get("type", type(event).__name__)
                    if isinstance(event, dict)
                    else type(event).__name__
                )
                logger.debug(f"Nova event received: {event_type}")
                if isinstance(event, dict) and event_type == "bidi_transcript_stream":
                    transcript = {
                        "role": event.get("role"),
                        "text": event.get("text") or event.get("current_transcript") or "",
                        "is_final": bool(event.get("is_final", False)),
                    }
                    publish_transcript(transcript)
                    if transcript["is_final"] and transcript["text"]:
                        logger.info(
                            f"Nova transcript [{transcript['role'] or 'unknown'}]: "
                            f"{transcript['text']}"
                        )
                await asyncio.gather(*[output(event) for output in outputs])
            logger.warn("Nova receive loop ended without an exception.")
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            logger.error(f"Nova receive loop failed: {type(exc).__name__}: {exc}")
            raise
        finally:
            inputs_task.cancel()

    logger.info("Starting explicit Nova Sonic agent loop...")
    inputs_task: asyncio.Task | None = None
    outputs_task: asyncio.Task | None = None
    try:
        await agent.start()
        logger.info("Nova Sonic agent.start() completed.")
        await start_io()
        inputs_task = asyncio.create_task(run_inputs())
        outputs_task = asyncio.create_task(run_outputs(inputs_task))
        done, pending = await asyncio.wait(
            {inputs_task, outputs_task}, return_when=asyncio.FIRST_EXCEPTION
        )
        for task in done:
            if task.cancelled():
                continue
            exc = task.exception()
            if exc is not None:
                raise exc
        logger.warn(
            "Nova agent IO tasks ended cleanly; this usually means the model stream closed."
        )
    finally:
        stopping_io.set()
        for task in (inputs_task, outputs_task):
            if task is not None and not task.done():
                task.cancel()
        await asyncio.gather(
            *[task for task in (inputs_task, outputs_task) if task is not None],
            return_exceptions=True,
        )
        try:
            await asyncio.wait_for(agent.stop(), timeout=8.0)
            logger.info("Nova Sonic agent.stop() completed.")
        except asyncio.TimeoutError:
            logger.warn("Timed out waiting for Nova Sonic agent.stop().")
        except Exception as exc:
            logger.warn(f"Nova Sonic agent.stop() failed: {type(exc).__name__}: {exc}")
        finally:
            await stop_io()


async def run_session(
    run,
    *,
    stop_event,
    activity,
    idle_timeout_seconds,
    max_session_seconds,
    heartbeat,
    logger,
    probe=None,
    poll_interval=1.0,
):
    """Supervise a session and join every task before handing audio back to wake detection."""

    async def idle_watch():
        while not stop_event.is_set():
            await asyncio.sleep(poll_interval)
            if idle_timeout_seconds > 0 and activity.idle_seconds() >= idle_timeout_seconds:
                stop_event.set()
                return "idle timeout"
        return "stop requested"

    run_task = asyncio.create_task(run)
    stop_task = asyncio.create_task(asyncio.to_thread(stop_event.wait))
    idle_task = asyncio.create_task(idle_watch())
    heartbeat_task = asyncio.create_task(heartbeat)
    max_task = asyncio.create_task(asyncio.sleep(max_session_seconds))
    tasks = {run_task, stop_task, idle_task, heartbeat_task, max_task}
    probe_task = asyncio.create_task(probe) if probe is not None else None
    try:
        done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        if run_task in done and not run_task.cancelled():
            exc = run_task.exception()
            if exc is not None:
                logger.error(f"Nova Sonic agent task failed: {type(exc).__name__}: {exc}")
        reason = "session ended"
        if max_task in done:
            reason = "max session duration"
        elif idle_task in done:
            try:
                reason = idle_task.result()
            except Exception:
                reason = "idle watcher ended"
        elif stop_task in done:
            reason = "stop requested"
        elif run_task in done:
            reason = "agent run completed"
        logger.info(f"Stopping Nova Sonic session: {reason}")
        return reason
    finally:
        stop_event.set()
        if probe_task is not None:
            tasks.add(probe_task)
        for task in tasks:
            if not task.done():
                task.cancel()
        results = await asyncio.gather(*tasks, return_exceptions=True)
        for result in results:
            if isinstance(result, Exception):
                logger.debug(f"Session task cleanup: {type(result).__name__}: {result}")
